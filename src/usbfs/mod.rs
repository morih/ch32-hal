//! USBFS - USB Full-Speedデバイスコントローラ (CH32X035およびその同系列チップ用)。
//!
//! このペリフェラルはDMAベースの設計になっている: 各エンドポイント番号ごとに専用の
//! SRAMバッファを持ち、そのアドレスを直接 `UEPn_DMA` レジスタに書き込む方式。
//! `usbd`(STM32互換)ドライバが使っている共有USBRAM/BTABLE方式とは異なる。
//!
//! EP0はコントロールエンドポイント。EP1〜EP3はクラスドライバが自由に使えるが、
//! それぞれ**単一方向専用**になる: 1つのエンドポイント番号にはDMAバッファが1つしかないため、
//! 同じエンドポイント番号をIN/OUT両方向で同時には使えない。CDC-ACMのように
//! データ用エンドポイントがちょうど3本(通知IN・バルクOUT・バルクIN)で足りるクラスなら
//! これで十分。EP4はハードウェア上EP0とDMAバッファを共有しているため、今のところ未対応。
//!
//! `usbfs-ep567` フィーチャーを有効にすると、独立したバッファを持つEP5〜EP7も追加で
//! 使えるようになる(CDC+HIDの複合デバイスなど、4本目以降のデータ用エンドポイントが
//! 必要な場合向け)。デフォルトでは無効: 有効にすると使わなくても静的バッファ・Waker配列の
//! 分だけRAMを常時消費するため、CDC単体などで足りる用途では無効のままにしておくとよい。

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver as driver;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType, Event, Unsupported,
};

use crate::gpio::Pull;
use crate::interrupt::typelevel::Interrupt;
use crate::pac::usb::regs::UepDma;
use crate::peripheral::RccPeripheral;
use crate::{interrupt, pac, Peri, PeripheralType};

/// このドライバが実装するハードウェアエンドポイント数。
///
/// `usbfs-ep567` フィーチャー無効時: EP0(コントロール) + EP1〜EP3 の4本。
/// 有効時: それに加えてEP5〜EP7も使えるようにするため8本ぶんの領域を確保する
/// (インデックス4=EP4はEP0とバッファ共有のため常に未使用のまま)。
#[cfg(not(feature = "usbfs-ep567"))]
const EP_COUNT: usize = 4;
#[cfg(feature = "usbfs-ep567")]
const EP_COUNT: usize = 8;
/// Full-Speedでの最大パケットサイズ(バイト数)。
const MAX_PACKET_SIZE: u16 = 64;

// UEPn_CTRL の T_RES/R_RES フィールドに書き込む値のエンコーディング。
// (ビット位置はレジスタ定義側で決まっているので、ここでは生の2bit値のみ定義する)
const RES_ACK: u8 = 0b00;
const RES_NAK: u8 = 0b10;
const RES_STALL: u8 = 0b11;

/// 4バイトアライメントされた固定長バッファ。USBのDMAはワード境界を要求するため、
/// 単なる `[u8; N]` ではなく明示的に `align(4)` を付けている。
#[repr(align(4))]
struct AlignedBuf<const N: usize>(UnsafeCell<[u8; N]>);
unsafe impl<const N: usize> Sync for AlignedBuf<N> {}
impl<const N: usize> AlignedBuf<N> {
    const fn new() -> Self {
        Self(UnsafeCell::new([0u8; N]))
    }
    fn as_ptr(&self) -> *mut u8 {
        self.0.get() as *mut u8
    }
}

// EP0〜EP3それぞれ専用の64バイトDMAバッファ。EP0はコントロール転送、EP1〜EP3は
// クラスドライバ(CDC-ACMなど)が使う。
static EP_BUF_0: AlignedBuf<64> = AlignedBuf::new();
static EP_BUF_1: AlignedBuf<64> = AlignedBuf::new();
static EP_BUF_2: AlignedBuf<64> = AlignedBuf::new();
static EP_BUF_3: AlignedBuf<64> = AlignedBuf::new();

// EP5〜EP7用の追加バッファ。`usbfs-ep567` フィーチャー有効時のみ確保する
// (無効時はこの分のRAMを一切消費しない)。
#[cfg(feature = "usbfs-ep567")]
static EP_BUF_5: AlignedBuf<64> = AlignedBuf::new();
#[cfg(feature = "usbfs-ep567")]
static EP_BUF_6: AlignedBuf<64> = AlignedBuf::new();
#[cfg(feature = "usbfs-ep567")]
static EP_BUF_7: AlignedBuf<64> = AlignedBuf::new();

/// エンドポイント番号からそのDMAバッファの先頭ポインタを返す。
fn ep_buf_ptr(index: usize) -> *mut u8 {
    match index {
        0 => EP_BUF_0.as_ptr(),
        1 => EP_BUF_1.as_ptr(),
        2 => EP_BUF_2.as_ptr(),
        3 => EP_BUF_3.as_ptr(),
        #[cfg(feature = "usbfs-ep567")]
        5 => EP_BUF_5.as_ptr(),
        #[cfg(feature = "usbfs-ep567")]
        6 => EP_BUF_6.as_ptr(),
        #[cfg(feature = "usbfs-ep567")]
        7 => EP_BUF_7.as_ptr(),
        _ => unreachable!(),
    }
}

/// 指定エンドポイントの`UEPn_CTRL`レジスタを返す。
///
/// EP0〜EP4は`UEP01234_CTRL(n)`(nはエンドポイント番号そのもの)、EP5〜EP7は
/// 別グループの`UEP567_CTRL(n)`(nは`index - 5`)という、ハードウェア側の2グループ構成を
/// ここで吸収し、呼び出し側は番号を意識せずに扱えるようにする。
fn ep_ctrl(usb: pac::usb::Usbd, index: usize) -> pac::common::Reg<pac::usb::regs::UepCtrl, pac::common::RW> {
    #[cfg(feature = "usbfs-ep567")]
    if index >= 5 {
        return usb.uep567_ctrl(index - 5);
    }
    usb.uep01234_ctrl(index)
}

/// 指定エンドポイントの`UEPn_T_LEN`(送信長)レジスタを返す。`ep_ctrl`と同じ理由で分岐する。
fn ep_t_len(usb: pac::usb::Usbd, index: usize) -> pac::common::Reg<pac::usb::regs::UepTLen, pac::common::RW> {
    #[cfg(feature = "usbfs-ep567")]
    if index >= 5 {
        return usb.uep567_t_len(index - 5);
    }
    usb.uep01234_t_len(index)
}

/// 指定エンドポイントのDMAアドレスレジスタを返す。EP0〜EP3は`UEP0123_DMA(n)`
/// (EP4はEP0と同じ`n=0`を共有するためここには出てこない)、EP5〜EP7は
/// `UEP567_DMA(n)`(n=index-5)を使う。
fn ep_dma(usb: pac::usb::Usbd, index: usize) -> pac::common::Reg<pac::usb::regs::UepDma, pac::common::RW> {
    #[cfg(feature = "usbfs-ep567")]
    if index >= 5 {
        return usb.uep567_dma(index - 5);
    }
    usb.uep0123_dma(index)
}

/// 指定エンドポイントのTX_EN/RX_ENビットを、そのエンドポイントが実際に属する
/// モードレジスタ(`UEP4_1_MOD`/`UEP2_3_MOD`/`UEP567_MOD`)に対して立てる。
///
/// EP1〜EP3は2つのレジスタ(UEP4_1_MOD, UEP2_3_MOD)を共有していて、それぞれ
/// 2エンドポイント分のビットを4bitずつ詰めて持っている(実機のレジスタ定義から
/// 逆算した対応関係: UEP4_1_MODのn=0はEP4・n=1はEP1、UEP2_3_MODのn=0はEP2・n=1はEP3)。
/// EP5〜EP7はUEP567_MODという別レジスタで、n=0,1,2がそれぞれEP5,6,7に対応する。
fn enable_ep_mode(usb: pac::usb::Usbd, index: usize, used_in: bool, used_out: bool) {
    match index {
        1 => usb.uep4_1_mod().modify(|w| {
            if used_in {
                w.set_tx_en(1, true);
            }
            if used_out {
                w.set_rx_en(1, true);
            }
        }),
        2 => usb.uep2_3_mod().modify(|w| {
            if used_in {
                w.set_tx_en(0, true);
            }
            if used_out {
                w.set_rx_en(0, true);
            }
        }),
        3 => usb.uep2_3_mod().modify(|w| {
            if used_in {
                w.set_tx_en(1, true);
            }
            if used_out {
                w.set_rx_en(1, true);
            }
        }),
        #[cfg(feature = "usbfs-ep567")]
        5 => usb.uep567_mod().modify(|w| {
            if used_in {
                w.set_tx_en(0, true);
            }
            if used_out {
                w.set_rx_en(0, true);
            }
        }),
        #[cfg(feature = "usbfs-ep567")]
        6 => usb.uep567_mod().modify(|w| {
            if used_in {
                w.set_tx_en(1, true);
            }
            if used_out {
                w.set_rx_en(1, true);
            }
        }),
        #[cfg(feature = "usbfs-ep567")]
        7 => usb.uep567_mod().modify(|w| {
            if used_in {
                w.set_tx_en(2, true);
            }
            if used_out {
                w.set_rx_en(2, true);
            }
        }),
        _ => unreachable!(),
    }
}

/// エンドポイント1つ分のDMAバッファへの読み書きヘルパー。
///
/// あえて生ポインタ(`*mut u8`)で持っている。これはUSBハードウェアのDMAが
/// 同じメモリ領域に非同期にアクセスするため、通常の `&mut` 参照として
/// 借用チェッカに扱わせるのは適切ではないから(volatileアクセスで直接触る)。
#[derive(Clone, Copy)]
struct EndpointBuffer {
    ptr: *mut u8,
    len: u16,
}
unsafe impl Send for EndpointBuffer {}
unsafe impl Sync for EndpointBuffer {}

impl EndpointBuffer {
    fn read(&self, buf: &mut [u8]) {
        assert!(buf.len() <= self.len as usize);
        for (i, b) in buf.iter_mut().enumerate() {
            *b = unsafe { core::ptr::read_volatile(self.ptr.add(i)) };
        }
    }

    fn write(&self, buf: &[u8]) {
        assert!(buf.len() <= self.len as usize);
        for (i, &b) in buf.iter().enumerate() {
            unsafe { core::ptr::write_volatile(self.ptr.add(i), b) };
        }
    }
}

const NEW_AW: AtomicWaker = AtomicWaker::new();
/// バス全体のイベント(PowerDetected/Reset/Suspend/Resume)を待っているタスクを起こすWaker。
static BUS_WAKER: AtomicWaker = NEW_AW;
/// エンドポイントごとのIN(デバイス→ホスト)完了待ちタスクを起こすWaker。
static EP_IN_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];
/// エンドポイントごとのOUT(ホスト→デバイス)完了待ちタスクを起こすWaker。
static EP_OUT_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];
/// 直近のOUT転送で実際に受信したバイト数。ISRが書き込み、`read()`側が読み出す。
static EP_OUT_LEN: [AtomicU16; EP_COUNT] = [const { AtomicU16::new(0) }; EP_COUNT];
/// EP0がSETUPパケットを受信済みで、まだ`setup()`が読み出していないことを示すフラグ。
static EP0_SETUP: AtomicBool = AtomicBool::new(false);
static IRQ_RESET: AtomicBool = AtomicBool::new(false);
static IRQ_SUSPEND: AtomicBool = AtomicBool::new(false);
static IRQ_RESUME: AtomicBool = AtomicBool::new(false);

/// 割り込みハンドラ本体。`bind_interrupts!` マクロ経由でUSBFS割り込みに紐付ける。
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    /// USBFS割り込みの本処理。
    ///
    /// ここでは「レジスタを更新してWakerを起こす」だけに留め、実際のプロトコル判断
    /// (ディスクリプタの内容やクラスの応答など)は一切行わない。それらは非同期タスク側
    /// (`Bus`/`ControlPipe`/`Endpoint`)が embassy-usb 経由で処理する。
    unsafe fn on_interrupt() {
        let usb = T::regs();
        let int_fg = usb.int_fg().read();

        if int_fg.transfer() {
            // 転送完了割り込み: どのエンドポイントの何トークンが完了したかを int_st から判定する。
            let int_st = usb.int_st().read();
            let index = int_st.mask_uis_endp() as usize;

            match int_st.mask_token() {
                0b11 => handle_setup(usb),
                0b10 if index < EP_COUNT => handle_in(usb, index),
                0b00 if index < EP_COUNT => handle_out(usb, index),
                _ => {} // 0b01 = SOF、または未対応のエンドポイント番号は無視する
            }

            usb.int_fg().write(|w| w.set_transfer(true)); // write-1-clear
        } else if int_fg.bus_rst() {
            usb.int_fg().write(|w| w.set_bus_rst(true));
            IRQ_RESET.store(true, Ordering::Relaxed);
            BUS_WAKER.wake();
        } else if int_fg.suspend() {
            usb.int_fg().write(|w| w.set_suspend(true));
            // mis_st.suspend() の値で「サスペンドに入った」のか「サスペンドから復帰した」のかを判定する。
            if usb.mis_st().read().suspend() {
                IRQ_SUSPEND.store(true, Ordering::Relaxed);
            } else {
                IRQ_RESUME.store(true, Ordering::Relaxed);
            }
            BUS_WAKER.wake();
        } else {
            // 想定外のフラグが立っていた場合はそのまま書き戻して(read-modify-writeではなく
            // 読んだ値をそのまま書く)、立っているフラグだけをクリアする。
            usb.int_fg().write_value(int_fg);
        }
    }
}

/// SETUPトークン受信時の処理。
///
/// SETUPの直後は仕様上、次に来るIN/OUTは必ずDATA1から始まる。また、SETUPを受け取った
/// 直後の時点ではまだどう応答すべきか(embassy-usb側の判断)が決まっていないので、
/// 両方向ともいったんNAKにしてホストを待たせておく。実際の応答は
/// `ControlPipe::setup()` がこのフラグを拾ってから、`accept()`/`data_in()`等が
/// 改めて組み立てる。
fn handle_setup(usb: pac::usb::Usbd) {
    usb.uep01234_ctrl(0).write(|w| {
        w.set_t_tog(true);
        w.set_r_tog(true);
        w.set_t_res(RES_NAK);
        w.set_r_res(RES_NAK);
    });
    EP0_SETUP.store(true, Ordering::Relaxed);
    EP_OUT_WAKERS[0].wake();
}

/// IN(デバイス→ホスト)転送完了時の処理。
///
/// 次回の送信に備えてデータトグルを反転し、応答をNAKに戻す(次のデータが
/// `write()`側から積まれるまでホストを待たせる)。
fn handle_in(usb: pac::usb::Usbd, index: usize) {
    ep_ctrl(usb, index).modify(|w| {
        let tog = !w.t_tog();
        w.set_t_tog(tog);
        w.set_t_res(RES_NAK);
    });
    EP_IN_WAKERS[index].wake();
}

/// OUT(ホスト→デバイス)転送完了時の処理。
///
/// 受信長(`rx_len`)は共有レジスタ1本しかなく、次の転送が来ると上書きされてしまうため、
/// ここで即座に読み取ってエンドポイントごとの変数に退避しておく。
fn handle_out(usb: pac::usb::Usbd, index: usize) {
    let rx_len = usb.rx_len().read().rx_len();
    EP_OUT_LEN[index].store(rx_len, Ordering::Relaxed);
    ep_ctrl(usb, index).modify(|w| {
        let tog = !w.r_tog();
        w.set_r_tog(tog);
        w.set_r_res(RES_NAK);
    });
    EP_OUT_WAKERS[index].wake();
}

/// エンドポイント1本分の割り当て状況。
#[derive(Clone, Copy)]
struct EndpointData {
    ep_type: EndpointType,
    used_in: bool,
    used_out: bool,
}

/// ボードのVDD電圧に応じたUSB PHYの動作モード。
///
/// D+のプルアップ抵抗値とPHYの耐圧設定は、VDDが3.3Vか5Vかで正しい組み合わせが変わる
/// (リファレンスマニュアルのAFIO_CTLRレジスタ、UDP_PUE/USB_PHY_V33の説明を参照)。
/// この2つは必ずセットで切り替える必要があるため、個別のビットではなくこのenumで渡す。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbVoltage {
    /// VDD ≈ 3.3V。D+に1.5kΩのプルアップ、PHYは3.3Vフル振幅モード。
    V33,
    /// VDD ≈ 5V。D+に10kΩのプルアップ、PHYは5Vトレラントモード。
    V5,
}

/// USBドライバ本体。`embassy_usb_driver::Driver` を実装する。
pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    alloc: [EndpointData; EP_COUNT],
}

impl<'d, T: Instance> Driver<'d, T> {
    /// 新しいUSBドライバを作成し、ハードウェアを初期化する。
    ///
    /// `dp`/`dm` はチップ固定でそれぞれ PC17/PC16 のみが対応する(`DpPin`/`DmPin` の
    /// 実装がこの2ピンにしか付いていない)。`vdd` は基板の実際のVDD電圧に合わせて指定する
    /// (`UsbVoltage`のドキュメント参照)。
    pub fn new(
        _usb: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: Peri<'d, impl DpPin<T>>,
        dm: Peri<'d, impl DmPin<T>>,
        vdd: UsbVoltage,
    ) -> Self {
        // D+/D- はフローティング入力のままにしておく: この後AFIOのUSB_IOENを立てると、
        // 以降はGPIOブロックではなくUSB PHYがこの2ピンを直接駆動するようになる。
        dp.set_as_input(Pull::None);
        dm.set_as_input(Pull::None);

        T::enable_and_reset();

        let usb = T::regs();

        pac::AFIO.ctlr().modify(|w| {
            w.set_udm_pue(0b00);
            match vdd {
                UsbVoltage::V33 => {
                    w.set_udp_pue(0b11); // D+に1.5kΩプルアップ(3.3V用)
                    w.set_usb_phy_v33(true); // 3.3Vフル振幅PHY
                }
                UsbVoltage::V5 => {
                    w.set_udp_pue(0b10); // D+に10kΩプルアップ(5V用)
                    w.set_usb_phy_v33(false); // 5Vトレラントモード
                }
            }
            w.set_usb_ioen(true); // D+/DMピンをGPIOからUSB PHYへ切り替える
        });

        usb.dev_ad().write(|w| w.set_mask_usb_addr(0));

        // 【重要・順序依存】プルアップ(ctrl.dev_pu_en)を有効にしてから udev_ctrl.port_en を
        // 有効にする必要がある。逆順にすると port_en が実際には立たず(読み戻すと0のまま)、
        // 受信はできるのに一切送信できない、という非常に分かりにくい壊れ方をする。
        // 実機でのハマりどころなので、順序を変えないこと。
        usb.ctrl().write(|w| {
            w.set_dev_pu_en(true);
            w.set_int_busy(true);
            w.set_dma_en(true);
        });

        usb.udev_ctrl().write(|w| {
            w.set_pd_dis(true);
            w.set_port_en(true);
        });

        usb.int_en().write(|w| {
            w.set_bus_rst(true);
            w.set_transfer(true);
            w.set_suspend(true);
        });

        Self {
            phantom: PhantomData,
            alloc: [EndpointData {
                ep_type: EndpointType::Bulk,
                used_in: false,
                used_out: false,
            }; EP_COUNT],
        }
    }

    /// 指定したエンドポイント番号がまだ未使用かどうか。
    ///
    /// EP0(インデックス0)はコントロール用に予約済み、EP4(インデックス4)はEP0との
    /// バッファ共有を実装していないため、どちらも対象外。それ以外(EP1〜EP3、
    /// `usbfs-ep567`有効時はEP5〜EP7も)は「1エンドポイントにつきDMAバッファ1つ」
    /// という制約上、IN/OUTどちらか片方でも使われていたらもう割り当てられない
    /// (モジュール冒頭のコメント参照)。
    fn is_endpoint_available(&self, index: usize) -> bool {
        index >= 1
            && index != 4
            && index < EP_COUNT
            && !self.alloc[index].used_in
            && !self.alloc[index].used_out
    }

    /// EP1〜EP3の中から空いている番号を探して割り当てる。IN/OUT共通の実装。
    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d, T, D>, EndpointAllocError> {
        if max_packet_size > MAX_PACKET_SIZE {
            return Err(EndpointAllocError);
        }

        let index = match ep_addr {
            Some(addr) if self.is_endpoint_available(addr.index()) => addr.index(),
            Some(_) => return Err(EndpointAllocError),
            None => (1..EP_COUNT)
                .find(|&i| self.is_endpoint_available(i))
                .ok_or(EndpointAllocError)?,
        };

        let ep = &mut self.alloc[index];
        ep.ep_type = ep_type;
        match D::dir() {
            Direction::In => ep.used_in = true,
            Direction::Out => ep.used_out = true,
        }

        Ok(Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, D::dir()),
                ep_type,
                max_packet_size,
                interval_ms,
            },
            buf: EndpointBuffer {
                ptr: ep_buf_ptr(index),
                len: MAX_PACKET_SIZE,
            },
        })
    }
}

impl<'d, T: Instance> driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, T, Out>;
    type EndpointIn = Endpoint<'d, T, In>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    /// `embassy_usb::Builder::build()` から呼ばれる。ここで初めて、実際に割り当てられた
    /// エンドポイントに対してDMAアドレスとTX_EN/RX_ENビットを設定し、割り込みを有効化する。
    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let usb = T::regs();

        for index in 1..EP_COUNT {
            if index == 4 {
                continue; // EP4はEP0とのバッファ共有を実装していないため常にスキップする
            }

            let ep = &self.alloc[index];
            if !ep.used_in && !ep.used_out {
                continue;
            }

            ep_dma(usb, index).write_value(UepDma(ep_buf_ptr(index) as u32));
            ep_ctrl(usb, index).write(|w| {
                w.set_t_res(RES_NAK);
                w.set_r_res(RES_NAK);
            });

            enable_ep_mode(usb, index, ep.used_in, ep.used_out);
        }

        usb.uep0123_dma(0).write_value(UepDma(ep_buf_ptr(0) as u32));
        usb.uep01234_ctrl(0).write(|w| {
            w.set_r_res(RES_ACK);
            w.set_t_res(RES_NAK);
        });

        unsafe {
            T::Interrupt::unpend();
            T::Interrupt::enable();
        }

        BUS_WAKER.wake();

        (
            Bus {
                phantom: PhantomData,
                inited: false,
            },
            ControlPipe {
                phantom: PhantomData,
                max_packet_size: control_max_packet_size,
            },
        )
    }
}

/// USBバス全体(接続/切断/リセット/サスペンド)を表すハンドル。
pub struct Bus<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    inited: bool,
}

impl<'d, T: Instance> driver::Bus for Bus<'d, T> {
    /// バスイベントを1つ待って返す。embassy-usbのメインループから繰り返し呼ばれる。
    async fn poll(&mut self) -> Event {
        poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());

            // 最初の1回だけは無条件に PowerDetected を返す(実際のVBUS検出はしていない)。
            if !self.inited {
                self.inited = true;
                return Poll::Ready(Event::PowerDetected);
            }

            if IRQ_RESET.load(Ordering::Acquire) {
                IRQ_RESET.store(false, Ordering::Relaxed);

                // バスリセットが起きたら、アドレスを0に戻し、全エンドポイントの応答状態を
                // 初期状態(EP0はSETUP/OUT受付可、他はNAK)にやり直す。
                let usb = T::regs();
                usb.dev_ad().write(|w| w.set_mask_usb_addr(0));
                usb.uep01234_ctrl(0).write(|w| {
                    w.set_r_res(RES_ACK);
                    w.set_t_res(RES_NAK);
                });
                for index in 1..EP_COUNT {
                    if index == 4 {
                        continue;
                    }
                    ep_ctrl(usb, index).write(|w| {
                        w.set_t_res(RES_NAK);
                        w.set_r_res(RES_NAK);
                    });
                }

                // 進行中だった転送はすべて仕切り直しになるので、待っているタスクを起こして
                // エラー(Disabled)として気づかせる。
                for w in &EP_IN_WAKERS {
                    w.wake();
                }
                for w in &EP_OUT_WAKERS {
                    w.wake();
                }

                return Poll::Ready(Event::Reset);
            }

            if IRQ_RESUME.load(Ordering::Acquire) {
                IRQ_RESUME.store(false, Ordering::Relaxed);
                return Poll::Ready(Event::Resume);
            }

            if IRQ_SUSPEND.load(Ordering::Acquire) {
                IRQ_SUSPEND.store(false, Ordering::Relaxed);
                return Poll::Ready(Event::Suspend);
            }

            Poll::Pending
        })
        .await
    }

    /// エンドポイントの有効/無効を切り替える。このハードウェアには明確な「無効状態」の
    /// レジスタ値が無いため、代わりにSTALLを「無効」の代用として使っている。
    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let usb = T::regs();
        let index = ep_addr.index();
        match ep_addr.direction() {
            Direction::In => {
                ep_ctrl(usb, index).modify(|w| w.set_t_res(if enabled { RES_NAK } else { RES_STALL }));
                EP_IN_WAKERS[index].wake();
            }
            Direction::Out => {
                ep_ctrl(usb, index).modify(|w| w.set_r_res(if enabled { RES_ACK } else { RES_STALL }));
                EP_OUT_WAKERS[index].wake();
            }
        }
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let usb = T::regs();
        let index = ep_addr.index();
        match ep_addr.direction() {
            Direction::In => {
                ep_ctrl(usb, index).modify(|w| w.set_t_res(if stalled { RES_STALL } else { RES_NAK }));
                EP_IN_WAKERS[index].wake();
            }
            Direction::Out => {
                ep_ctrl(usb, index).modify(|w| w.set_r_res(if stalled { RES_STALL } else { RES_ACK }));
                EP_OUT_WAKERS[index].wake();
            }
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let usb = T::regs();
        let ctrl = ep_ctrl(usb, ep_addr.index()).read();
        match ep_addr.direction() {
            Direction::In => ctrl.t_res() == RES_STALL,
            Direction::Out => ctrl.r_res() == RES_STALL,
        }
    }

    async fn enable(&mut self) {}
    async fn disable(&mut self) {}

    /// リモートウェイクアップは未対応。
    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

trait Dir {
    fn dir() -> Direction;
}

/// "IN" 方向(デバイス→ホスト)を表すマーカー型。
pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

/// "OUT" 方向(ホスト→デバイス)を表すマーカー型。
pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}

/// USBエンドポイント(コントロール以外、EP1〜EP3のいずれか。`usbfs-ep567`有効時はEP5〜EP7も)。
pub struct Endpoint<'d, T: Instance, D> {
    _phantom: PhantomData<(&'d mut T, D)>,
    info: EndpointInfo,
    buf: EndpointBuffer,
}

impl<'d, T: Instance, D> driver::Endpoint for Endpoint<'d, T, D> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    /// エンドポイントがSTALL状態から解除される(＝使える状態になる)まで待つ。
    async fn wait_enabled(&mut self) {
        let index = self.info.addr.index();
        let dir = self.info.addr.direction();
        poll_fn(|cx| {
            match dir {
                Direction::In => EP_IN_WAKERS[index].register(cx.waker()),
                Direction::Out => EP_OUT_WAKERS[index].register(cx.waker()),
            }
            let usb = T::regs();
            let ctrl = ep_ctrl(usb, index).read();
            let disabled = match dir {
                Direction::In => ctrl.t_res() == RES_STALL,
                Direction::Out => ctrl.r_res() == RES_STALL,
            };
            if disabled {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }
}

impl<'d, T: Instance> driver::EndpointOut for Endpoint<'d, T, Out> {
    /// ホストからのOUTデータを1パケット分読み取る。
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let index = self.info.addr.index();

        // r_resがNAK(=ISRが受信済みでデータを保持している)になるまで待つ。
        // STALLだった場合はDisabledエラーとして呼び出し側に伝える。
        let stat = poll_fn(|cx| {
            EP_OUT_WAKERS[index].register(cx.waker());
            let usb = T::regs();
            let ctrl = ep_ctrl(usb, index).read();
            match ctrl.r_res() {
                RES_NAK => Poll::Ready(RES_NAK),
                RES_STALL => Poll::Ready(RES_STALL),
                _ => Poll::Pending,
            }
        })
        .await;

        if stat == RES_STALL {
            return Err(EndpointError::Disabled);
        }

        let rx_len = EP_OUT_LEN[index].load(Ordering::Relaxed) as usize;
        if rx_len > buf.len() {
            return Err(EndpointError::BufferOverflow);
        }
        self.buf.read(&mut buf[..rx_len]);

        // 読み出しが終わったので次のOUTパケットを受け付けられるようにACKへ戻す。
        let usb = T::regs();
        ep_ctrl(usb, index).modify(|w| w.set_r_res(RES_ACK));

        Ok(rx_len)
    }
}

impl<'d, T: Instance> driver::EndpointIn for Endpoint<'d, T, In> {
    /// ホストへ1パケット分のデータを送信する。
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        let index = self.info.addr.index();

        // 前回の送信が完了してt_resがNAKに戻るまで待つ(=次の送信を積める状態になるまで)。
        let stat = poll_fn(|cx| {
            EP_IN_WAKERS[index].register(cx.waker());
            let usb = T::regs();
            let ctrl = ep_ctrl(usb, index).read();
            match ctrl.t_res() {
                RES_NAK => Poll::Ready(RES_NAK),
                RES_STALL => Poll::Ready(RES_STALL),
                _ => Poll::Pending,
            }
        })
        .await;

        if stat == RES_STALL {
            return Err(EndpointError::Disabled);
        }

        self.buf.write(buf);

        let usb = T::regs();
        ep_t_len(usb, index).write(|w| w.set_t_len(buf.len() as u8));
        ep_ctrl(usb, index).modify(|w| w.set_t_res(RES_ACK));

        Ok(())
    }
}

/// コントロール転送(EP0)専用のパイプ。EP0はIN/OUT両方向を同じ1つのDMAバッファで
/// 使い回す(コントロール転送はSETUP→データ→ステータスと段階が順番に進むだけで、
/// 双方向が同時に動くことはないため、バッファ共有で問題ない)。
pub struct ControlPipe<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    max_packet_size: u16,
}

impl<'d, T: Instance> ControlPipe<'d, T> {
    fn buf(&self) -> EndpointBuffer {
        EndpointBuffer {
            ptr: ep_buf_ptr(0),
            len: MAX_PACKET_SIZE,
        }
    }

    /// EP0のステータスステージ用ZLP(ゼロ長パケット)を送信し、完了を待つ。
    ///
    /// 戻り値が `false` の場合は、ZLPが実際に完了する前に**新しいSETUPパケットが
    /// 割り込んできた**ことを意味する(ホストが制御転送を最初からやり直した、など)。
    /// この場合、呼び出し側(`accept_set_address`など)は本来の処理(アドレス確定等)を
    /// 実行してはいけない――仕様上、ステータスステージが正常完了するまでは
    /// 効果を確定させてはならないため。
    async fn send_status_zlp(&mut self) -> bool {
        let usb = T::regs();
        usb.uep01234_t_len(0).write(|w| w.set_t_len(0));
        usb.uep01234_ctrl(0).modify(|w| w.set_t_res(RES_ACK));

        poll_fn(|cx| {
            EP_IN_WAKERS[0].register(cx.waker());
            if EP0_SETUP.load(Ordering::Relaxed) {
                return Poll::Ready(false);
            }
            let ctrl = T::regs().uep01234_ctrl(0).read();
            if ctrl.t_res() == RES_NAK {
                Poll::Ready(true)
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d, T: Instance> driver::ControlPipe for ControlPipe<'d, T> {
    fn max_packet_size(&self) -> usize {
        self.max_packet_size as usize
    }

    /// 次のSETUPパケットを待って、その8バイトを返す。
    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|cx| {
            EP_OUT_WAKERS[0].register(cx.waker());
            if EP0_SETUP.load(Ordering::Relaxed) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        EP0_SETUP.store(false, Ordering::Relaxed);

        let mut buf = [0u8; 8];
        self.buf().read(&mut buf);
        buf
    }

    /// コントロールOUT(ホスト→デバイス)のデータステージを1チャンク読み取る。
    async fn data_out(&mut self, buf: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        let usb = T::regs();
        usb.uep01234_ctrl(0).modify(|w| w.set_r_res(RES_ACK));

        poll_fn(|cx| {
            EP_OUT_WAKERS[0].register(cx.waker());
            // 待っている間に新しいSETUPが来たら、今の転送は打ち切られたということなので即終了する。
            if EP0_SETUP.load(Ordering::Relaxed) {
                return Poll::Ready(true);
            }
            let ctrl = T::regs().uep01234_ctrl(0).read();
            if ctrl.r_res() == RES_NAK {
                Poll::Ready(false)
            } else {
                Poll::Pending
            }
        })
        .await;

        if EP0_SETUP.load(Ordering::Relaxed) {
            return Err(EndpointError::Disabled);
        }

        let rx_len = EP_OUT_LEN[0].load(Ordering::Relaxed) as usize;
        if rx_len > buf.len() {
            return Err(EndpointError::BufferOverflow);
        }
        self.buf().read(&mut buf[..rx_len]);

        Ok(rx_len)
    }

    /// コントロールIN(デバイス→ホスト)のデータステージを1チャンク送信する。
    async fn data_in(&mut self, data: &[u8], _first: bool, last: bool) -> Result<(), EndpointError> {
        if data.len() > self.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        self.buf().write(data);

        let usb = T::regs();
        usb.uep01234_t_len(0).write(|w| w.set_t_len(data.len() as u8));
        usb.uep01234_ctrl(0).modify(|w| w.set_t_res(RES_ACK));

        poll_fn(|cx| {
            EP_IN_WAKERS[0].register(cx.waker());
            if EP0_SETUP.load(Ordering::Relaxed) {
                return Poll::Ready(());
            }
            let ctrl = T::regs().uep01234_ctrl(0).read();
            if ctrl.t_res() == RES_NAK {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        if EP0_SETUP.load(Ordering::Relaxed) {
            return Err(EndpointError::Disabled);
        }

        if last {
            // 最後のチャンクなら、締めくくりのステータスステージ(ホストからのOUT ZLP)を
            // 受け付けられるようにr_resをACKにしておく。
            T::regs().uep01234_ctrl(0).modify(|w| w.set_r_res(RES_ACK));
        }

        Ok(())
    }

    /// データステージの無いリクエスト(またはOUTデータステージの後)を正常受理し、
    /// ステータスステージのZLPを送って完了させる。
    async fn accept(&mut self) {
        self.send_status_zlp().await;
    }

    /// 未対応のリクエストをSTALLで拒否する。
    async fn reject(&mut self) {
        let usb = T::regs();
        usb.uep01234_ctrl(0).modify(|w| {
            w.set_t_res(RES_STALL);
            w.set_r_res(RES_STALL);
        });
    }

    /// SET_ADDRESSリクエストを受理する。
    ///
    /// USB仕様上、デバイスアドレスの変更はステータスステージ(ZLP)が正常に完了した
    /// **後**でなければならない。`send_status_zlp()` が `false`(=ZLP完了前に次の
    /// SETUPで打ち切られた)を返した場合は、アドレスを書き換えずに諦める。
    async fn accept_set_address(&mut self, addr: u8) {
        let ok = self.send_status_zlp().await;
        if ok {
            let usb = T::regs();
            usb.dev_ad().modify(|w| w.set_mask_usb_addr(addr));
        }
    }
}

trait SealedInstance {
    fn regs() -> pac::usb::Usbd;
}

/// USBFSインスタンス(現状はUSBFSペリフェラル1つのみ)を表すトレイト。
#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccPeripheral + PeripheralType + 'static {
    /// このUSBインスタンスに対応する割り込み。
    type Interrupt: interrupt::typelevel::Interrupt;
}

// build.rsが生成する `foreach_interrupt!` マクロで、metapac側のペリフェラル定義
// (kind="usb", signal="GLOBAL")にマッチするインスタンスへ自動的に実装を生やす。
foreach_interrupt!(
    ($inst:ident, usb, $block:ident, GLOBAL, $irq:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> pac::usb::Usbd {
                unsafe { pac::usb::Usbd::from_ptr(crate::pac::$inst.as_ptr()) }
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
);

/// D+ピンであることを示すトレイト。
pub trait DpPin<T: Instance>: crate::gpio::Pin {}
/// D-ピンであることを示すトレイト。
pub trait DmPin<T: Instance>: crate::gpio::Pin {}

// CH32X035系はD+/D-のピンがハードウェア固定(PC17=D+, PC16=D-)なので、
// build.rsの汎用コード生成には頼らず、ここで直接実装する。
#[cfg(ch32x0)]
impl DpPin<crate::peripherals::USBFS> for crate::peripherals::PC17 {}
#[cfg(ch32x0)]
impl DmPin<crate::peripherals::USBFS> for crate::peripherals::PC16 {}
