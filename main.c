/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_LIS3DSH.h"
#include "eeprom.h"
#include "stm32l1xx_hal.h"

#include "math.h"
#include <stdlib.h>

#define EEPROM_START_ADDRESS 0x08080000

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */



const float PI = 3.141592, filtre_carpani = 0.1; // SABİT TANIMLAMALARI

uint8_t hambuffer[2], secim=0;  // GENEL DEGİSKENLER

float x=0, y=0, Xham=0, Yham=0, Zham=0, tam_tur=0, sifir_derece_degerit=0; // GENEL DEGİSKENLER

float sifir_derece_degerix=0, sifirlanmis_rollx =0, sifirlama_farkix=0;   // X EKSENİ SIFIRLAMA DEGİSKENLERİ
float sifir_derece_degeriy=0, sifirlanmis_pitchy=0, sifirlama_farkiy=0;   // Y EKSENİ SIFIRLAMA DEGİSKENLERİ

float kalmanrollx =0, kalman_oldx=0, cov_oldx=0, kalman_newx=0, cov_newx=0, kalman_gainx=0, kalman_calculatedx=0; // X EKSENİ KALMAN FİLTRESİ DEGİSKENLERİ
float kalmanpitchy=0, kalman_oldy=0, cov_oldy=0, kalman_newy=0, cov_newy=0, kalman_gainy=0, kalman_calculatedy=0; // Y EKSENİ KALMAN FİLTRESİ DEGİSKENLERİ
float kalman_oldz =0, cov_oldz=0, kalman_newz=0, cov_newz=0, kalman_gainz=0, kalman_calculatedz=0; 				  // Z EKSENİ KALMAN FİLTRESİ DEGİSKENLERİ

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config (void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC_Init (void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

LIS3DSH_DataScaled myData;

float kalman_filterx (float inputx)  // X EKSENİ KALMAN FİLTRESİ
{
  kalman_newx = kalman_oldx;  // eski değer alınır
  cov_newx    = cov_oldx + filtre_carpani; // yeni kovaryans değeri belirlenir. filtre çarpanı=0.10 alınmıştır

  kalman_gainx       = cov_newx / (cov_newx + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
  kalman_calculatedx = kalman_newx + (kalman_gainx * (inputx - kalman_newx)); //kalman değeri hesaplanır
  kalman_oldx        = kalman_calculatedx;

  cov_newx = (1 - kalman_gainx) * cov_oldx; //yeni kovaryans değeri hesaplanır
  cov_oldx = cov_newx; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir

  return kalman_calculatedx; //hesaplanan kalman değeri çıktı olarak verilir
}


float kalman_filtery (float inputy)  // Y EKSENİ KALMAN FİLTRESİ
{
  kalman_newy = kalman_oldy;     // eski değer alınır
  cov_newy    = cov_oldy + filtre_carpani; // yeni kovaryans değeri belirlenir. filtre çarpanı=0.10 alınmıştır

  kalman_gainy       = cov_newy / (cov_newy + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
  kalman_calculatedy = kalman_newy + (kalman_gainy * (inputy - kalman_newy)); //kalman değeri hesaplanır
  kalman_oldy        = kalman_calculatedy;

  cov_newy = (1 - kalman_gainy) * cov_oldy; //yeni kovaryans değeri hesaplanır
  cov_oldy = cov_newy; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir

  return kalman_calculatedy; //hesaplanan kalman değeri çıktı olarak verilir
}


float kalman_filterz (float inputz)  // Z EKSENİ KALMAN FİLTRESİ
{
  kalman_newz = kalman_oldz;     // eski değer alınır
  cov_newz    = cov_oldz + filtre_carpani; // yeni kovaryans değeri belirlenir. filtre çarpanı=0.10 alınmıştır

  kalman_gainz       = cov_newz / (cov_newz + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
  kalman_calculatedz = kalman_newz + (kalman_gainz * (inputz - kalman_newz)); //kalman değeri hesaplanır
  kalman_oldz        = kalman_calculatedz;

  cov_newz = (1 - kalman_gainz) * cov_oldz; //yeni kovaryans değeri hesaplanır
  cov_oldz = cov_newz; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir

  return kalman_calculatedz; //hesaplanan kalman değeri çıktı olarak verilir
}


void ivme_olc(void) // mG TÜRÜNDEN SENSÖR VERİSİ OKUMA
{
    if(LIS3DSH_PollDRDY(1000) == true)
    {
      myData = LIS3DSH_GetDataScaled();

      // 1000 mG DE SINIRLAMA
      if(myData.x > 1000) myData.x = 1000;
      if(myData.x <-1000) myData.x =-1000;
      if(myData.y > 1000) myData.y = 1000;
      if(myData.y <-1000) myData.y =-1000;
      if(myData.z > 1000) myData.z = 1000;
      if(myData.z <-1000) myData.z =-1000;
    }
}



void verileri_filtrele(void) // KALMAN FİLTRESİ İLE TİTREŞİM GÜRÜLTÜLERİ ÖNLEME
{
  myData.x = kalman_filterx(myData.x);
  myData.y = kalman_filtery(myData.y);
  myData.z = kalman_filterz(myData.z);
}


void dereceye_cevir(void) // +-1000 mG GELEN SENSÖR DEGERİNİ İSTENİLEN EGİM AÇISINA DÖNÜŞTÜRME
{
  kalmanrollx  = atan(myData.x / sqrt(pow(myData.y, 2) + pow(myData.z, 2))) * 180 / PI + 90; // X ekseni 0 - 180 derece dönüştürme
  kalmanpitchy = atan(myData.y / sqrt(pow(myData.x, 2) + pow(myData.z, 2))) * 180 / PI + 90; // Y ekseni 0 - 180 derece dönüştürme

  tam_tur = atan2(myData.y, myData.x) * 180 / PI + 180; //  X in Y ye göre  0 - 360 Derece dönüştürme
  // tam_tur = atan2(myData.x, myData.z) * 180 / PI + 180; // X ekseni için 0 - 360 Derece dönüştürme
  // tam_tur = atan2(myData.y, myData.z) * 180 / PI + 180; // Y ekseni için 0 - 360 Derece dönüştürme
}



void sifirlama(void) // EXT. TEACH ( SIFIRLAMA )
{
  if(HAL_GPIO_ReadPin(TEACH_GPIO_Port, TEACH_Pin) == 0)
  {
    secim++;

	if(secim == 1) // Sıfırlama seçildi ise
	{
	  HAL_Delay(700);

	  sifir_derece_degerit = tam_tur;
	  sifir_derece_degerix = kalmanrollx;
	  sifir_derece_degeriy = kalmanpitchy;

	  eeprom_write(EEPROM_START_ADDRESS, secim);
	  save_data(EEPROM_START_ADDRESS+1,  EEPROM_START_ADDRESS+5 , sifir_derece_degerix);
	  save_data(EEPROM_START_ADDRESS+6,  EEPROM_START_ADDRESS+10, sifir_derece_degeriy);
	  save_data(EEPROM_START_ADDRESS+11, EEPROM_START_ADDRESS+15, sifir_derece_degerit);

	  HAL_Delay(700);
	}

	else // Sıfırlama seçilmedi ise
	{
	  HAL_Delay(700);

	  sifir_derece_degerix = 0;
	  sifir_derece_degeriy = 0;
	  sifirlanmis_rollx    = 0;
	  sifirlanmis_pitchy   = 0;
	  secim                = 0;

	  eeprom_write( EEPROM_START_ADDRESS,secim );
	  save_data( EEPROM_START_ADDRESS+1, EEPROM_START_ADDRESS+5 , sifir_derece_degerix );
	  save_data( EEPROM_START_ADDRESS+6, EEPROM_START_ADDRESS+10, sifir_derece_degeriy );

	  HAL_Delay(700);
	}
  }
}

// CIKIS FONKSİYONLARI

void akim_cikis_180deg_4_20mA(void)
{
  if(secim == 0) // NORMAL CIKIS
  {
    x = ((kalmanrollx  * 18.2) + 816); // 4 - 20 mA için +-90 Dereceyi DAC Çıkışına Oranlama
    y = ((kalmanpitchy * 18.2) + 816); // 4 - 20 mA için +-90 Dereceyi DAC Çıkışına Oranlama
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = kalmanrollx  - sifir_derece_degerix;
    sifirlama_farkiy = kalmanpitchy - sifir_derece_degeriy;

    sifirlanmis_rollx  = 90 + sifirlama_farkix;
    sifirlanmis_pitchy = 90 + sifirlama_farkiy;

    if(sifirlanmis_rollx  > 180) sifirlanmis_rollx  = 180;
    if(sifirlanmis_pitchy > 180) sifirlanmis_pitchy = 180;
    if(sifirlanmis_rollx  < 0  ) sifirlanmis_rollx  = 0  ;
    if(sifirlanmis_pitchy < 0  ) sifirlanmis_pitchy = 0  ;

    x = ((sifirlanmis_rollx  * 18.2) + 816); // 4 - 20 mA için +-90 Dereceyi DAC Çıkışına Oranlama
    y = ((sifirlanmis_pitchy * 18.2) + 816); // 4 - 20 mA için +-90 Dereceyi DAC Çıkışına Oranlama
  }
}


void akim_cikis_180deg_0_20mA(void)
{
  if(secim == 0) // NORMAL CIKIS
  {
    x = kalmanrollx  * 22.8; // 0 - 20 mA için +-90 Dereceyi DAC Çıkışına Oranlama
    y = kalmanpitchy * 22.8; // 0 - 20 mA için +-90 Dereceyi DAC Çıkışına Oranlama
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = kalmanrollx  - sifir_derece_degerix;
    sifirlama_farkiy = kalmanpitchy - sifir_derece_degeriy;

    sifirlanmis_rollx  = 90 + sifirlama_farkix;
    sifirlanmis_pitchy = 90 + sifirlama_farkiy;

    if(sifirlanmis_rollx  > 180) sifirlanmis_rollx  = 180;
    if(sifirlanmis_pitchy > 180) sifirlanmis_pitchy = 180;
    if(sifirlanmis_rollx  < 0  ) sifirlanmis_rollx  = 0  ;
    if(sifirlanmis_pitchy < 0  ) sifirlanmis_pitchy = 0  ;

    x = sifirlanmis_rollx  * 22.8; // 0 - 20 mA için +-90 Dereceyi DAC Çıkışına Oranlama
    y = sifirlanmis_pitchy * 22.8; // 0 - 20 mA için +-90 Dereceyi DAC Çıkışına Oranlama
  }
}


void voltaj_cikis_180deg_0_10V(void)
{
  if(secim == 0) // NORMAL CIKIS
  {
    x = kalmanrollx  * 22.8; // 0 - 10 V için +-90 Dereceyi DAC Çıkışına Oranlama
	y = kalmanpitchy * 22.8; // 0 - 10 V için +-90 Dereceyi DAC Çıkışına Oranlama
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = kalmanrollx  - sifir_derece_degerix;
    sifirlama_farkiy = kalmanpitchy - sifir_derece_degeriy;

    sifirlanmis_rollx  = 90 + sifirlama_farkix;
    sifirlanmis_pitchy = 90 + sifirlama_farkiy;

    if(sifirlanmis_rollx  > 180) sifirlanmis_rollx  = 180;
    if(sifirlanmis_pitchy > 180) sifirlanmis_pitchy = 180;
    if(sifirlanmis_rollx  < 0  ) sifirlanmis_rollx  = 0  ;
    if(sifirlanmis_pitchy < 0  ) sifirlanmis_pitchy = 0  ;


    x = sifirlanmis_rollx  * 22.8; // 0 - 10 V için +-90 Dereceyi DAC Çıkışına Oranlama
    y = sifirlanmis_pitchy * 22.8; // 0 - 10 V için +-90 Dereceyi DAC Çıkışına Oranlama
  }
}


void voltaj_cikis_180deg_0_5V(void)
{
  if(secim == 0) // NORMAL CIKIS
  {
    x = kalmanrollx  * 11.55; // 0 - 10 V için +-90 Dereceyi DAC Çıkışına Oranlama
    y = kalmanpitchy * 11.55; // 0 - 10 V için +-90 Dereceyi DAC Çıkışına Oranlama
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = kalmanrollx  - sifir_derece_degerix;
    sifirlama_farkiy = kalmanpitchy - sifir_derece_degeriy;

    sifirlanmis_rollx  = 90 + sifirlama_farkix;
    sifirlanmis_pitchy = 90 + sifirlama_farkiy;

    if(sifirlanmis_rollx  > 180) sifirlanmis_rollx  = 180;
    if(sifirlanmis_pitchy > 180) sifirlanmis_pitchy = 180;
    if(sifirlanmis_rollx  < 0  ) sifirlanmis_rollx  = 0  ;
    if(sifirlanmis_pitchy < 0  ) sifirlanmis_pitchy = 0  ;

    x = sifirlanmis_rollx  * 11.55; // 0 - 10 V için +-90 Dereceyi DAC Çıkışına Oranlama
    y = sifirlanmis_pitchy * 11.55; // 0 - 10 V için +-90 Dereceyi DAC Çıkışına Oranlama
  }
}


void voltaj_cikis_180deg_05_45V(void)
{
  if(secim == 0) // NORMAL CIKIS
  {
    x = (( kalmanrollx  ) * 9.21) + 204; // 0.5 - 4.5 V için +-90 Dereceyi DAC Çıkışına Oranlama
    y = (( kalmanpitchy ) * 9.21) + 204; // 0.5 - 4.5 V için +-90 Dereceyi DAC Çıkışına Oranlama
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = kalmanrollx  - sifir_derece_degerix;
    sifirlama_farkiy = kalmanpitchy - sifir_derece_degeriy;

    sifirlanmis_rollx  = 90 + sifirlama_farkix;
    sifirlanmis_pitchy = 90 + sifirlama_farkiy;

    if(sifirlanmis_rollx  > 180) sifirlanmis_rollx  = 180;
    if(sifirlanmis_pitchy > 180) sifirlanmis_pitchy = 180;
    if(sifirlanmis_rollx  < 0  ) sifirlanmis_rollx  = 0  ;
    if(sifirlanmis_pitchy < 0  ) sifirlanmis_pitchy = 0  ;

    x = (( sifirlanmis_rollx  ) * 9.21) + 204; // 0.5 - 4.5 V için +-90 Dereceyi DAC Çıkışına Oranlama
    y = (( sifirlanmis_pitchy ) * 9.21) + 204; // 0.5 - 4.5 V için +-90 Dereceyi DAC Çıkışına Oranlama
  }
}


void voltaj_cikis_360deg_0_5V(void)
{
  if(secim == 0) // NORMAL CIKIS
  {
    x = tam_tur * 5.75; // 0 - 5V için 360 Dereceyi DAC Çıkışına Oranlama
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = tam_tur  - sifir_derece_degerit;

    if(sifirlama_farkix  > 360)
    {
      sifirlama_farkix  = sifirlama_farkix - 360 ;
    }

    if(sifirlama_farkix  < 0  )
    {
      sifirlama_farkix  = sifirlama_farkix + 360  ;
    }
    x = sifirlama_farkix * 5.75; // 0 - 5V için 360 Dereceyi DAC Çıkışına Oranlama
  }
}


void voltaj_cikis_360deg_05_45V(void)
{
  if(secim == 0) // NORMAL CIKIS
  {
    x = ( tam_tur * 4.61) + 204; // 0.5 - 4.5V için 360 Dereceyi DAC Çıkışına Oranlama
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = tam_tur  - sifir_derece_degerit;

    if(sifirlama_farkix  > 360)
    {
      sifirlama_farkix  = sifirlama_farkix - 360 ;
    }

    if(sifirlama_farkix  < 0  )
    {
      sifirlama_farkix  = sifirlama_farkix + 360  ;
    }
    x = (sifirlama_farkix * 4.61) + 204; // 0.5 - 4.5V için 360 Dereceyi DAC Çıkışına Oranlama
  }
}


void voltaj_cikis_360deg_0_10V(void)
{

  if(secim == 0) // NORMAL CIKIS
  {
    x = tam_tur * 11.375; // 0 - 10 V için 360 Dereceyi DAC Çıkışına Oranlama
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = tam_tur  - sifir_derece_degerit;

    if(sifirlama_farkix  > 360)
    {
      sifirlama_farkix  = sifirlama_farkix - 360 ;
    }

    if(sifirlama_farkix  < 0  )
    {
      sifirlama_farkix  = sifirlama_farkix + 360  ;
    }
    x = sifirlama_farkix * 11.375; // 4 - 20 mA için 360 Dereceyi DAC Çıkışına Oranlama
  }
}


void akim_cikis_360deg_4_20mA(void)
{

  if(secim == 0) // NORMAL CIKIS
  {
    x = ((tam_tur) * 9.1) + 819; // 4 - 20 mA için 360 Dereceyi DAC Çıkışına Oranlama
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = tam_tur  - sifir_derece_degerit;

    if(sifirlama_farkix  > 360)
    {
      sifirlama_farkix  = sifirlama_farkix - 360 ;
    }

    if(sifirlama_farkix  < 0  )
    {
      sifirlama_farkix  = sifirlama_farkix + 360  ;
    }
    x = (sifirlama_farkix * 9.1) + 819; // 4 - 20 mA için 360 Dereceyi DAC Çıkışına Oranlama
  }
}


void akim_cikis_360deg_0_20mA(void)
{

  if(secim == 0) // NORMAL CIKIS
  {
    // 0 - 20 mA için 360 Dereceyi DAC Çıkışına Oranlama
    x = tam_tur * 11.375;
  }

  else // SIFIRLANMIS CIKIS
  {
    sifirlama_farkix = tam_tur  - sifir_derece_degerit;

    if(sifirlama_farkix  > 360)
    {
      sifirlama_farkix  = sifirlama_farkix - 360 ;
    }
    if(sifirlama_farkix  < 0  )
    {
      sifirlama_farkix  = sifirlama_farkix + 360  ;
    }

    // 0 - 20 mA için 360 Dereceyi DAC Çıkışına Oranlama
    x = sifirlama_farkix * 11.375;
  }
}


void ham_deger (void) // SENSÖR HAM DEGERLERİ OKUMA
{
  //Read X data
  LIS3DSH_ReadIO(LIS3DSH_OUT_X_L_ADDR, hambuffer, 2);
  Xham = ((hambuffer[1] << 8) + hambuffer[0]);

  //Read Y data
  LIS3DSH_ReadIO(LIS3DSH_OUT_Y_L_ADDR, hambuffer, 2);
  Yham = ((hambuffer[1] << 8) + hambuffer[0]);

  //Read Z data
  LIS3DSH_ReadIO(LIS3DSH_OUT_Z_L_ADDR, hambuffer, 2);
  Zham = ((hambuffer[1] << 8) + hambuffer[0]);
}


void dac_yaz(void)
{
  if(x > 4095) x = 4095;
  if(y > 4095) x = 4095;

  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,  x); // X ÇIKIŞ
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,  y); // Y ÇIKIŞ
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	LIS3DSH_InitTypeDef myAccConfigDef;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_DAC_Init ();
  /* USER CODE BEGIN 2 */

  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

  HAL_Delay(500);

  myAccConfigDef.dataRate        = LIS3DSH_DATARATE_50;
  myAccConfigDef.fullScale       = LIS3DSH_FULLSCALE_2;
  myAccConfigDef.antiAliasingBW  = LIS3DSH_FILTER_BW_50;
  myAccConfigDef.enableAxes      = LIS3DSH_XYZ_ENABLE;
  myAccConfigDef.interruptEnable = false;
  LIS3DSH_Init(&hspi1, &myAccConfigDef);

  LIS3DSH_X_calibrate(-1000.0 , 1000.0);
  LIS3DSH_Y_calibrate(-1000.0, 1000.0);
  LIS3DSH_Z_calibrate(-1000.0 , 1000.0);

  // EEPROM dan sifirlama degerlerini oku
  secim = eeprom_read(EEPROM_START_ADDRESS);
  sifir_derece_degerix = read_data( EEPROM_START_ADDRESS+1,  EEPROM_START_ADDRESS+5  );
  sifir_derece_degeriy = read_data( EEPROM_START_ADDRESS+6,  EEPROM_START_ADDRESS+10 );
  sifir_derece_degerit = read_data( EEPROM_START_ADDRESS+11, EEPROM_START_ADDRESS+15 );


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  ivme_olc();

  verileri_filtrele();

  dereceye_cevir();

  sifirlama();

  // YALNIZ BIR CIKIS SECILMELI

  // voltaj_cikis_180deg_0_10V();
  // voltaj_cikis_180deg_0_5V();
  // voltaj_cikis_180deg_05_45V();
  // voltaj_cikis_360deg_0_10V();
  // voltaj_cikis_360deg_0_5V();
  // voltaj_cikis_360deg_05_45V();
  // akim_cikis_180deg_4_20mA();
  // akim_cikis_180deg_0_20mA();
  // akim_cikis_360deg_4_20mA();
   akim_cikis_360deg_0_20mA();

	dac_yaz();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CS1_Pin CS2_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TEACH_Pin */
  GPIO_InitStruct.Pin = TEACH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TEACH_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
