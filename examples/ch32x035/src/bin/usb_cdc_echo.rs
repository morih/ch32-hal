//! CH32X035用 USB CDC-ACM エコーの例。ch32-halの `usbfs` ドライバ + embassy-usb を使う。
//!
//! D+はPC17、D-はPC16(ハードウェア固定)。1つのCDC-ACM仮想COMポートとして列挙され、
//! 送られてきたバイト列をそのままエコーバックする。

#![no_std]
#![no_main]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use hal::usbfs::{Driver, Instance};
use hal::{bind_interrupts, peripherals};
use panic_halt as _;

// USBFSペリフェラルの割り込みを、ch32-halが生成する型付き割り込みハンドラに紐付ける。
bind_interrupts!(struct Irqs {
    USBFS => hal::usbfs::InterruptHandler<peripherals::USBFS>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    // USBFSのクロックはHCLKをそのまま使う(分周器を持たない)ため、Full-Speed(12Mbps)の
    // タイミングを正しく出すにはHCLKをちょうど48MHzにする必要がある。
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI,
        ..Default::default()
    });

    // D+/D-ピンはハードウェア固定でPC17/PC16のみが使える。
    let driver = Driver::new(p.USBFS, Irqs, p.PC17, p.PC16);

    let mut config = embassy_usb::Config::new(0x1a86, 0xfe0c);
    config.manufacturer = Some("ch32-rs");
    config.product = Some("CH32X035 USB CDC echo");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Windows側でCDC-ACMとして正しく認識させるための設定。
    // composite_with_iadsをtrueにする場合はdevice_class等を0xEF/0x02/0x01の組み合わせに
    // しなければならない(embassy-usb側の制約)。ここではCDC-ACM単体構成なのでfalseのまま、
    // クラス自体のコード(0x02/0x02/0x00)を使う。
    config.device_class = 0x02;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x00;
    config.composite_with_iads = false;

    // ディスクリプタ組み立て用のワークバッファ。CDC-ACM 1クラス分なので256バイトあれば十分。
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // MSOSディスクリプタは使わない
        &mut control_buf,
    );

    // CDC-ACMクラスを1つ登録する。内部でエンドポイントを3本(通知IN・データOUT・データIN)
    // 確保する ―― ちょうどこのドライバが持つEP1〜EP3の数と一致する。
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // build()の時点でドライバのDriver::start()が呼ばれ、エンドポイントのDMAアドレスや
    // 割り込みが実際に有効化される。
    let mut usb = builder.build();
    let usb_fut = usb.run();

    // ホストがCDCのDTR(接続)を立てるたびにエコー処理を繰り返す。
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            let _ = echo(&mut class).await;
        }
    };

    // usb_futとecho_futは同じタスク上で協調動作するので、どちらか一方が長時間
    // ブロックするとUSB全体の処理が止まってしまう点に注意(このタスク内で重い処理や
    // ブロッキング待ちをしないこと)。
    embassy_futures::join::join(usb_fut, echo_fut).await;
}

/// ホスト切断などでエコー処理を中断したことを表すマーカー型。
struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

/// 接続中、受信したパケットをそのまま送り返し続ける。
///
/// ホストが切断すると `read_packet`/`write_packet` が `EndpointError::Disabled` を返すので、
/// それを `Disconnected` に変換してループを抜け、呼び出し元(`main`)で次の接続を待ち直す。
async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        class.write_packet(data).await?;
    }
}
