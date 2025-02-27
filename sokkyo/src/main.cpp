// /* ヘッダーファイル読み込み */
// #include "mbed.h"

// /* ピンの機能設定  */
// BufferedSerial PC (USBTX,USBRX,115200);            //USB :シリアル通信
// DigitalOut USSTriger (PE_2);         //P11 :超音波センサ トリガ出力
// Timer ActiveTime;

// /* 割り込み処理宣言 */
// Ticker TrigerTiming;                //Trigerピン :インターバルタイマ
// InterruptIn USSEcho (PE_4);          //p12 :超音波センサ  エコー入力

// /* 関数宣言 */
// void init(void);
// void Output_Monitor(unsigned short Value);

// /* グローバル変数宣言*/
// unsigned short USSDistance;         //USSDistance:超音波センサ測定距離

// /* main関数開始*/
// int main() {
//                          //val:PC.readable初期化用変数
//     init();
//     while(1) {
//         if( PC.readable() ){
//             Output_Monitor( USSDistance );
//         }
//     }
// }


// /***************************************************
//  * @brief       60ms毎の割り込みでUSSTrigerに10usのON出力
//  * @param       なし
//  * @return      なし
//  * @date 2014/12/16 新規作成
//  **************************************************/
// void Triger (){
//     USSTriger = 1;
//     wait_us(10);
//     USSTriger = 0;
// }

// /***************************************************
//  * @brief       USSEcho立ち上がりでの割り込み
//  * @brief       Hiの場合ActiveTimeタイマスタート
//  * @param       なし
//  * @return      なし
//  * @date 2014/12/16 新規作成
//  **************************************************/
// void RiseEcho(){
//     ActiveTime.start();
// }

// /***************************************************
//  * @brief       USSEcho立ち下がりでの割り込み
//  * @brief       Lowの場合ActiveTimeタイマ停止+値読み取り
//  * @param       なし
//  * @return      なし
//  * @date 2014/12/16 新規作成
//  **************************************************/
// void FallEcho(){
//     unsigned long ActiveWidth;
//     ActiveTime.stop();
//     ActiveWidth = ActiveTime.read_us();
//     USSDistance = ActiveWidth * 0.0170;
//     ActiveTime.reset();
// }

// /***************************************************
//  * @brief       各種機能のプロパティ設定
//  * @param       なし
//  * @return      なし
//  * @date 2014/12/13 新規作成
//  **************************************************/
// void init(void){   
//     TrigerTiming.attach( Triger , 0.060 );      //USSTriger周期 60ms
//     USSEcho.rise( RiseEcho );                   //USSEcho立ち上がり時割り込み
//     USSEcho.fall( FallEcho );                   //USSEcho立ち下がり時割り込み
// }

// /***************************************************
//  * @brief       Parameterの値をPC画面に出力
//  * @param       Value : 画面に出力する値
//  * @return      なし
//  * @date 2014/12/14 新規作成
//  **************************************************/
// void Output_Monitor(unsigned short Value){
//     printf("%d[cm]\r\n",Value);
// }

/*
RRST NHK2025
IPアドレスは適宜変更すること
垂直MD基板用にピンを変更
エンコーダーから計算した変位と速度をUDPで送信する
2025/02/05
*/

#include "EthernetInterface.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>
#include <vector>

#define PI 3.141592653589793

float v[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 速度の格納[mm/s]
float d[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 変位[m]

void receive(UDPSocket *receiver);

// マッピング関数
int map(int value, int inMin, int inMax, int outMin, int outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// PWM
PwmOut MD1P(PA_0);
PwmOut MD2P(PA_3);
PwmOut MD3P(PC_7);
PwmOut MD4P(PC_6);
PwmOut MD5P(PC_8);
PwmOut MD6P(PC_9);

// DIR
DigitalOut MD1D(PD_2);
DigitalOut MD2D(PG_2);
DigitalOut MD3D(PD_5);
DigitalOut MD4D(PD_6);
DigitalOut MD5D(PD_7);
DigitalOut MD6D(PC_10);

//サーボ
PwmOut SERVO1(PB_1);
PwmOut SERVO2(PB_6);
PwmOut SERVO3(PD_13);
PwmOut SERVO4(PB_8);


//トランジスタ（電磁弁・表示灯用）
DigitalOut TR1(PF_0);
DigitalOut TR2(PF_1);
DigitalOut TR3(PC_10);
DigitalOut TR4(PC_11);
DigitalOut TR5(PC_12);
DigitalOut TR6(PF_14);
DigitalOut TR7(PF_12);
DigitalOut TR8(PF_13);

// CAN
CAN can{PD_0, PD_1, (int)1e6}; // rd,td,1Mhz

double mdd[7]; // MDに出力する方向指令を格納
double mdp[7]; // MDに出力するduty比を格納

const char *recievefromIP = nullptr; //ネットワーク切断検知用

int main() {

  // PWM周波数の設定
  MD1P.period_us(50);
  MD2P.period_us(50);
  MD3P.period_us(50);
  MD4P.period_us(50);
  MD5P.period_us(50);
  MD6P.period_us(50);
  /*
  50(us) = 1000(ms) / 20000(Hz) * 10^3
  MDに合わせて調整
  CytronのMDはPWM周波数が20kHzなので上式になる
  */


  //サーボのPWM周波数の設定
  SERVO1.period_ms(20);
  SERVO2.period_ms(20);
  SERVO3.period_ms(20);
  SERVO4.period_ms(20);

  // 送信先情報
  const char *destinationIP = "192.168.0.195";
  const uint16_t destinationPort = 4000;

  // 自機情報
  //const char *myIP = "192.168.0.215"; // MR
  const char *myIP = "192.168.0.218"; // DR
  //const char *myIP = "192.168.128.215"; // DR on test
  const char *myNetMask = "255.255.255.0";
  const uint16_t receivePort = 5000;

  //クラスのインスタンス化
  EthernetInterface net;
  SocketAddress destination, source, myData;
  UDPSocket udp;
  Thread receiveThread;

  //DHCPオフ（IPは固定）
  net.set_dhcp(false);
  net.set_network(myIP, myNetMask, "");

  printf("Start\n");

  // マイコンをネットワークに接続
  if (net.connect() != 0) {
    printf("Network connection Error >_<\n");
    return -1;
  } else {
    printf("Network connection success ^_^\n");
  }

  // UDPソケットをオープン
  udp.open(&net);

  // portをバインドする
  udp.bind(receivePort);

  // 送信先の情報を入力
  destination.set_ip_address(destinationIP);
  destination.set_port(destinationPort);

  // 受信用のスレッドをスタート
  receiveThread.start(callback(receive, &udp));

  // メインループ（送信用）
  while (1) {
    using namespace std::chrono;
   
    // 速度データをカンマ区切りの文字列に変換
    char sendData[128]; // 送信データを格納する配列
    snprintf(sendData, sizeof(sendData), "%f,%f,%f,%f,%f,%f,%f,%f,", v[1], v[2],
             v[3], v[4], d[1], d[2], d[3], d[4]);

    //送信データを表示（デバッグ用）
    // printf("Sending (%d bytes): %s\n", strlen(sendData), sendData);

    // UDP送信
    if (const int result =
            udp.sendto(destination, sendData, strlen(sendData)) < 0) {
      printf("send Error: %d\n", result); // エラー処理
    }

  }

  // スレッドの終了を待つ
  receiveThread.join();

  // UDPソケットを閉じ、ネットワーク接続を切断
  udp.close();
  net.disconnect();
  return 0;
}

void receive(UDPSocket *receiver) { // UDP受信スレッド

  using namespace std::chrono;

  SocketAddress source;
  std::vector<int16_t> data(19, 0); // 19要素の整数ベクトルを0で初期化

  while (1) {

    int recv_size = receiver->recvfrom(&source, reinterpret_cast<char*>(data.data()), data.size() * sizeof(int));
    if (recv_size < 0) {
        printf("Receive Error: %d\n", recv_size);
        continue;
    }

    // recievefromIP = source.get_ip_address();
    // printf("Received %d bytes from %s\n", recv_size, recievefromIP);
   

      //方向成分と速度成分を分離
      for (int i = 1; i <= 6; i++) {
        if (data[i] >= 0) {
          mdd[i] = 1;
        } else {
          mdd[i] = 0;
        }
        mdp[i] = fabs((data[i]) / 100.0);
      }

    SERVO1.pulsewidth_us(map(data[7], 0, 270, 500, 2500));
    SERVO2.pulsewidth_us(map(data[8], 0, 270, 500, 2500));
    SERVO3.pulsewidth_us(map(data[9], 0, 270, 500, 2500));
    SERVO4.pulsewidth_us(map(data[10], 0, 270, 500, 2500));

    if(data[0]){
      printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8],
        data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18]);
    }


    // MDに出力
    MD1D = mdd[1];
    MD2D = mdd[2];
    MD3D = mdd[3];
    MD4D = mdd[4];
    MD5D = mdd[5];
    MD6D = mdd[6];

    MD1P = mdp[1];
    MD2P = mdp[2];
    MD3P = mdp[3];
    MD4P = mdp[4];
    MD5P = mdp[5];
    MD6P = mdp[6];

    // トランジスタに出力
    TR1 = data[11];
    TR2 = data[12];
    TR3 = data[13];
    TR4 = data[14];
    TR5 = data[15];
    TR6 = data[16];
    TR7 = data[17];
    TR8 = data[18];
  }
}