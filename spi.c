/*************************************************************************
 * File         :spi.c                       				 
 * Date         :2021/10/10					 
 * Summary      :Use for spi communication			
 * CPU type     :PIC16F886						 
 * Compiler     :XC8							 
 * Author       :						 
 * Description  :		 
 *              :				 
 *              :		 
 *              :			 
 * ***********************************************************************

/*---File Include---*/
#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <spi.h>

/*---Initiation---*/
void spi_init(void)
{
    SSPCON = _SSPCON_CONFIG;
    SSPSTAT = _SSPSTAT_CONFIG;
    INTCONbits.GIE = 1;                                                         //äÑÇËçûÇ›ê›íË
    INTCONbits.PEIE = 1;                                                        //äÑÇËçûÇ›ê›íË
    PIE1bits.SSPIE = 0;                                                         //SSPäÑÇËçûÇ›ÇÉCÉlÅ[ÉuÉã
    PIR1bits.SSPIF = 0;                                                         //SSPäÑÇËçûÇ›ÉtÉâÉOÇÉNÉäÉA
    TRISAbits.TRISA4 = 0;                                                       //CSÉsÉìÇèoóÕê›íË
    TRISAbits.TRISA5 = 0;                                                       //CSÉsÉìÇèoóÕê›íË
    TRISCbits.TRISC3 = 0;                                                       //CLKÉsÉìÇèoóÕê›íË
    TRISCbits.TRISC4 = 1;                                                       //DIÉsÉìÇì¸óÕê›íË
    TRISCbits.TRISC5 = 0;                                                       //DOÉsÉìÇèoóÕê›íË
    PORTAbits.RA4 = 1;                                                          //CSÉsÉìÇHighÇ≈ë“ã@
    PORTAbits.RA5 = 1;                                                          //CSÉsÉìÇHighÇ≈ë“ã@
}

/*---Communication---*/
char spi_putc(char c)
{
    char dumy;                                                                  //É_É~Å[ÉäÅ[ÉhópïœêîêÈåæ
    char data;                                                                  //ÉfÅ[É^äiî[ópïœêîêÈåæ
    
    dumy = SSPBUF;                                                              //ダミー読み出し                                                          
    if(!SSPSTATbits.BF){                                                        //バッファが空だったら
        SSPBUF = c;                                                             //送信
    }
    while(!SSPSTATbits.BF);                                                     //受信完了するまで待つ
    data = SSPBUF;                                                              //受信データ取得
    
    //__delay_us(1);
    return data;                                                                //受信データを返す
}

