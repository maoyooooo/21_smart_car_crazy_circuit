/*********************************************************************************************************************
* TC264 Opensourec Library 鍗筹紙TC264 寮�婧愬簱锛夋槸涓�涓熀浜庡畼鏂� SDK 鎺ュ彛鐨勭涓夋柟寮�婧愬簱
* Copyright (c) 2022 SEEKFREE 閫愰绉戞妧
*
* 鏈枃浠舵槸 TC264 寮�婧愬簱鐨勪竴閮ㄥ垎
*
* TC264 寮�婧愬簱 鏄厤璐硅蒋浠�
* 鎮ㄥ彲浠ユ牴鎹嚜鐢辫蒋浠跺熀閲戜細鍙戝竷鐨� GPL锛圙NU General Public License锛屽嵆 GNU閫氱敤鍏叡璁稿彲璇侊級鐨勬潯娆�
* 鍗� GPL 鐨勭3鐗堬紙鍗� GPL3.0锛夋垨锛堟偍閫夋嫨鐨勶級浠讳綍鍚庢潵鐨勭増鏈紝閲嶆柊鍙戝竷鍜�/鎴栦慨鏀瑰畠
*
* 鏈紑婧愬簱鐨勫彂甯冩槸甯屾湜瀹冭兘鍙戞尌浣滅敤锛屼絾骞舵湭瀵瑰叾浣滀换浣曠殑淇濊瘉
* 鐢氳嚦娌℃湁闅愬惈鐨勯�傞攢鎬ф垨閫傚悎鐗瑰畾鐢ㄩ�旂殑淇濊瘉
* 鏇村缁嗚妭璇峰弬瑙� GPL
*
* 鎮ㄥ簲璇ュ湪鏀跺埌鏈紑婧愬簱鐨勫悓鏃舵敹鍒颁竴浠� GPL 鐨勫壇鏈�
* 濡傛灉娌℃湁锛岃鍙傞槄<https://www.gnu.org/licenses/>
*
* 棰濆娉ㄦ槑锛�
* 鏈紑婧愬簱浣跨敤 GPL3.0 寮�婧愯鍙瘉鍗忚 浠ヤ笂璁稿彲鐢虫槑涓鸿瘧鏂囩増鏈�
* 璁稿彲鐢虫槑鑻辨枃鐗堝湪 libraries/doc 鏂囦欢澶逛笅鐨� GPL3_permission_statement.txt 鏂囦欢涓�
* 璁稿彲璇佸壇鏈湪 libraries 鏂囦欢澶逛笅 鍗宠鏂囦欢澶逛笅鐨� LICENSE 鏂囦欢
* 娆㈣繋鍚勪綅浣跨敤骞朵紶鎾湰绋嬪簭 浣嗕慨鏀瑰唴瀹规椂蹇呴』淇濈暀閫愰绉戞妧鐨勭増鏉冨０鏄庯紙鍗虫湰澹版槑锛�
*
* 鏂囦欢鍚嶇О          cpu0_main
* 鍏徃鍚嶇О          鎴愰兘閫愰绉戞妧鏈夐檺鍏徃
* 鐗堟湰淇℃伅          鏌ョ湅 libraries/doc 鏂囦欢澶瑰唴 version 鏂囦欢 鐗堟湰璇存槑
* 寮�鍙戠幆澧�          ADS v1.10.2
* 閫傜敤骞冲彴          TC264D
* 搴楅摵閾炬帴          https://seekfree.taobao.com/
*
* 淇敼璁板綍
* 鏃ユ湡              浣滆��                澶囨敞
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 灏嗘湰璇彞涓�#pragma section all restore璇彞涔嬮棿鐨勫叏灞�鍙橀噺閮芥斁鍦–PU0鐨凴AM涓�

// 鏈緥绋嬫槸寮�婧愬簱绌哄伐绋� 鍙敤浣滅Щ妞嶆垨鑰呮祴璇曞悇绫诲唴澶栬
// 鏈緥绋嬫槸寮�婧愬簱绌哄伐绋� 鍙敤浣滅Щ妞嶆垨鑰呮祴璇曞悇绫诲唴澶栬
// 鏈緥绋嬫槸寮�婧愬簱绌哄伐绋� 鍙敤浣滅Щ妞嶆垨鑰呮祴璇曞悇绫诲唴澶栬

// **************************** 浠ｇ爜鍖哄煙 ****************************
#define IPS200_TYPE     (IPS200_TYPE_SPI)                                       // 鎺掗拡 骞跺彛涓ゅ灞� 杩欓噷瀹忓畾涔夊～鍐� IPS200_TYPE_PARALLEL8
                                                                                      // 鎺掗拡 SPI 涓ゅ灞� 杩欓噷瀹忓畾涔夊～鍐� IPS200_TYPE_SPI

int core0_main(void)
{
    clock_init();                   // 鑾峰彇鏃堕挓棰戠巼<鍔″繀淇濈暀>
    debug_init();                   // 鍒濆鍖栭粯璁よ皟璇曚覆鍙�
    // 姝ゅ缂栧啓鐢ㄦ埛浠ｇ爜 渚嬪澶栬鍒濆鍖栦唬鐮佺瓑
    //zf_log(mt9v03x_double_init(mt9v03x_1), "mt9v03x init error");
    //ips200_init(IPS200_TYPE);
    //wireless_uart_init();
    Motor_Init();              // 鐢垫満鍒濆鍖�
    EncoderInit();            // 缂栫爜鍣ㄥ垵濮嬪寲
        
        
    pit_ms_init(CCU60_CH0, 10); // 鍒濆鍖� PIT 瀹氭椂鍣� 10ms 涓柇涓�娆�
    // 姝ゅ缂栧啓鐢ㄦ埛浠ｇ爜 渚嬪澶栬鍒濆鍖栦唬鐮佺瓑
    cpu_wait_event_ready();         // 绛夊緟鎵�鏈夋牳蹇冨垵濮嬪寲瀹屾瘯
    while (TRUE)
    {
        // 姝ゅ缂栧啓闇�瑕佸惊鐜墽琛岀殑浠ｇ爜
        //Image_Processing_And_Get_Center();
        //Calculate_All_Errors();
        //Send_Image_To_ZF_Assistant();
        //ips200_displayimage03x(mt9v03x_image_1, MT9V03X_1_W, MT9V03X_1_H); // 鏄剧ず鎽勫儚澶村浘鍍忓埌灞忓箷涓� 浠呰皟璇曠敤
        //Show_Image_On_IPS200(); // 鏄剧ず浜屽�煎寲鍥惧儚鍜岃竟绾垮埌灞忓箷涓� 浠呰皟璇曠敤
        // 姝ゅ缂栧啓闇�瑕佸惊鐜墽琛岀殑浠ｇ爜
//        Set_Motor_PWM(25.0f, 25.0f); // 鐩存帴璁剧疆涓�涓浐瀹氱殑PWM杈撳嚭 浠呮祴璇曠敤
        //printf("current_L_rpm: %f\r\n", current_L_rpm);
        printf("current_R_rpm: %f\r\n", current_R_rpm);
        //printf("current_L_rpm: %f\r\n", current_L_rpm);
        system_delay_ms(1);
    }
}

#pragma section all restore
// **************************** 浠ｇ爜鍖哄煙 ****************************
