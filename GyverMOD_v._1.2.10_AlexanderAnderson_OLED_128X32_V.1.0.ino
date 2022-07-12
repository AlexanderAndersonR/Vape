/*
  Created 2017  by AlexGyver  AlexGyver Home Labs Inc.
  ВНИМАНИЕ! ПУТЬ К ПАПКЕ СО СКЕТЧЕМ НЕ ДОЛЖЕН СОДЕРЖАТЬ РУССКИХ СИМВОЛОВ
  ВО ИЗБЕЖАНИЕ ПРОБЛЕМ ПОЛОЖИТЕ ПАПКУ В КОРЕНЬ ДИСКА С

  Внимание! При первом запуске initial_calibration должен быть равен 1 (строка №17)
  При подключении и открытии монитора порта будет запущен процесс калибровки.
  Вам нужно при помощи вольтметра измерить напряжение на пинах 5V и GND,
  затем отправить его в монитор В МИЛЛИВОЛЬТАХ, т.е. если на вольтметре 4.56
  то отправить примерно 4560. После этого изменить initial_calibration на 0
  и заново прошить Arduino.
  Если хотите пропустить процесс калибровки, то введите то же самое напряжение,
  что было показано вам при калибровке (real VCC). И снова прошейте код.
  
  Added new display OLED 128x32 by Alexander Anderson 2022 
*/
//-----------------------------------НАСТРОЙКИ------------------------------------
#define initial_calibration 0/////  // калибровка вольтметра 1 - включить, 0 - выключить
#define welcome 1              // приветствие (слова GYVER VAPE при включении), 1 - включить, 0 - выключить
#define battery_info 1         // отображение напряжения аккумулятора при запуске, 1 - включить, 0 - выключить
#define sleep_timer 30         // таймер сна в секундах
#define vape_threshold 4       // отсечка затяжки, в секундах
#define turbo_mode 0           // турбо режим 1 - включить, 0 - выключить
#define battery_percent 0      // отображать заряд в процентах, 1 - включить, 0 - выключить
#define battery_low 2.8        // нижний порог срабатывания защиты от переразрядки аккумулятора, в Вольтах!
//-----------------------------------НАСТРОЙКИ------------------------------------

#include <EEPROMex.h>   // библиотека для работы со внутренней памятью ардуино
#include <LowPower.h>   // библиотека сна

//-----------кнопки-----------
#define butt_up 5      // кнопка вверх
#define butt_down 4    // кнпока вниз
#define butt_set 3     // кнопка выбора
#define butt_vape 2    // кнопка "парить"
//-----------кнопки-----------

//-----------флажки-----------
boolean up_state, down_state, set_state, vape_state;
boolean up_flag, down_flag, set_flag, set_flag_hold, set_hold, vape_btt, vape_btt_f;
volatile boolean wake_up_flag, vape_flag;
boolean change_v_flag, change_w_flag, change_o_flag;
volatile byte mode, mode_flag = 1;
boolean flag;          // флаг, разрешающий подать ток на койл (защита от КЗ, обрыва, разрядки)
//-----------флажки-----------

//-----------пины-------------
#define mosfet 10      // пин мосфета (нагрев спирали)
#define battery 6      // пин измерения напряжения акума
//-----------пины-------------

//-----------дисплей-----------
#include <TimerOne.h>
//-----------дисплей-----------
//-----------дисплей OLED 128x32-----------
#include <Adafruit_GFX.h>        //OLED библиотека
#include <Adafruit_SSD1306.h>
#include <Wire.h>                //Библиотека I2C
#define SCREEN_WIDTH 128 // Ширина OLED-дисплея, в пикселях
#define SCREEN_HEIGHT 32 // Высота OLED-дисплея в пикселях
#define OLED_RESET    -1 // т.к.у дисплея нет пина сброса прописываем -1, если используется общий сброс Arduino
#define disp_vcc 12
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Объявляем имя и задаем параметры
//-----------дисплей OLED 128x32-----------

int bat_vol, bat_volt_f;   // хранит напряжение на акуме
int PWM, PWM_f;           // хранит PWM сигнал

//-------переменные и коэффициенты для фильтра-------
int bat_old, PWM_old = 800;
float filter_k = 0.04;
float PWM_filter_k = 0.1;
//-------переменные и коэффициенты для фильтра-------

unsigned long last_time, vape_press, set_press, last_vape, wake_timer; // таймеры
int volts, watts;    // храним вольты и ватты
float ohms;          // храним омы
float my_vcc_const;  // константа вольтметра
volatile byte vape_mode, vape_release_count;


void setup() {
  Serial.begin(9600);
  //---настройка кнопок и выходов-----
  pinMode(butt_up , INPUT_PULLUP);
  pinMode(butt_down , INPUT_PULLUP);
  pinMode(butt_set , INPUT_PULLUP);
  pinMode(butt_vape , INPUT_PULLUP);
  pinMode(mosfet , OUTPUT);
  pinMode(disp_vcc , OUTPUT);
  digitalWrite(disp_vcc, HIGH);
  Timer1.disablePwm(mosfet);    // принудительно отключить койл
  digitalWrite(mosfet, LOW);    // принудительно отключить койл
  //---настройка кнопок и выходов-----
  if (initial_calibration) calibration();  // калибровка, если разрешена

  //----читаем из памяти-----
  volts = EEPROM.readInt(0);
  watts = EEPROM.readInt(2);
  ohms = EEPROM.readFloat(4);
  my_vcc_const = EEPROM.readFloat(8);
  //----читаем из памяти-----

  //----инициализируем дисплей-----
  delay(2000);                              //Пауза для инизиализации дисплея
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Запуск дисплея
  display.display();
  
  //----инициализируем дисплей old-----
  Timer1.initialize(1500);          // таймер
  Timer1.attachInterrupt(timerIsr);

  //------приветствие-----
  if (welcome) {
    display.clearDisplay();               //Очистить дисплей
    display.setTextSize(3);               //Задаем размер текста
    display.setTextColor(WHITE);          //Задаем цвет текста
    display.setCursor(0,0);             //Задаем координату начала текста в пикселях
    display.println("WELCOME");             //Пишем сам текст  
    display.display();                    //Команда для отображения всего этого на дисплее
    delay(700);
    display.clearDisplay();               
    display.setTextSize(2);               
    display.setTextColor(WHITE);          
    display.setCursor(0,0);             
    display.println("Alexander");             
    display.display();    
    display.setCursor(0,15);            
    display.println("Anderson");           
    display.display(); 
    delay(2000);
  }
  //------приветствие-----

  // измерить напряжение аккумулятора
  bat_vol = readVcc();
  bat_old = bat_vol;

  // проверка заряда акума, если разряжен то прекратить работу
  if (bat_vol < battery_low * 1000) {
    flag = 0;
    //disp.clear();
    //disp_send(LOWB);
    display.clearDisplay();               
    display.setTextSize(3);               
    display.setTextColor(WHITE);          
    display.setCursor(0,0);             
    display.println("BAT LOW");            
    display.display();       
    Timer1.disablePwm(mosfet);    // принудительно отключить койл
    digitalWrite(mosfet, LOW);    // принудительно отключить койл
    delay(1000);
  } else {
    flag = 1;
  }

  if (battery_info) {  // отобразить заряд аккумулятора при первом включении
    display.clearDisplay();              
    display.setTextSize(3);               
    display.setTextColor(WHITE);         
    display.setCursor(0,0);            
    display.println(String((float)bat_vol / 1000));              
    display.display();       
    delay(1000);
  }
}

void loop() {
  if (millis() - last_time > 50) {                       // 20 раз в секунду измеряем напряжение
    last_time = millis();
    bat_vol = readVcc();                                 // измерить напряжение аккумулятора в миллиВольтах
    bat_volt_f = filter_k * bat_vol + (1 - filter_k) * bat_old;  // фильтруем
    bat_old = bat_volt_f;                                // фильтруем
    if (bat_volt_f < battery_low * 1000) {               // если напряжение меньше минимального
      flag = 0;                                          // прекратить работу
       display.clearDisplay();              
       display.setTextSize(3);               
       display.setTextColor(WHITE);          
       display.setCursor(0,0);            
       display.println("BAT LOW");            
       display.display();       
      Timer1.disablePwm(mosfet);    // принудительно отключить койл
      digitalWrite(mosfet, LOW);  // принудительно отключить койл
      delay(1000);
    }
  }

  //-----------опрос кнопок-----------
  up_state = !digitalRead(butt_up);
  down_state = !digitalRead(butt_down);
  set_state = !digitalRead(butt_set);
  vape_state = !digitalRead(butt_vape);

  // если нажата любая кнопка, "продлить" таймер ухода в сон
  if (up_state || down_state || set_state || vape_state) wake_timer = millis();
  //-----------опрос кнопок-----------

  // service_mode();  // раскомментировать для отладки кнопок
  // показывает, какие кнопки нажаты или отпущены
  // использовать для проерки правильности подключения

  //---------------------отработка нажатия SET и изменение режимов---------------------
  if (flag) {                              // если акум заряжен
    if (set_state && !set_hold) {          // если кнпока нажата
      set_hold = 1;
      set_press = millis();                // начинаем отсчёт
      while (millis() - set_press < 300) {
        if (digitalRead(butt_set)) {       // если кнопка отпущена до 300 мс
          set_hold = 0;
          set_flag = 1;
          break;
        }
      }
    }
    if (set_hold && set_state) {           // если кнопка всё ещё удерживается
      if (!set_flag_hold) {
        display.clearDisplay();  
        set_flag_hold = 1;
      }
      if (round(millis() / 150) % 2 == 0) {
        if (!battery_percent) {
          display.clearDisplay();               
          display.setTextSize(3);               
          display.setTextColor(WHITE);          
          display.setCursor(0,0);            
          display.println(String((float)bat_volt_f / 1000));             
          display.display();   
        } else {
          display.clearDisplay();               
          display.setTextSize(3);              
          display.setTextColor(WHITE);          
          display.setCursor(0,0);            
          display.println(String(map(bat_volt_f, battery_low * 1000, 4200, 0, 99)));              
          display.display(); 
        }
      }
    }
    if (set_hold && !set_state && set_flag_hold) {  // если удерживалась и была отпущена
      set_hold = 0;
      set_flag_hold = 0;
      mode_flag = 1;
    }

    if (!set_state && set_flag) {  // если нажали-отпустили
      set_hold = 0;
      set_flag = 0;
      mode++;                      // сменить режим
      mode_flag = 1;
      if (mode > 2) mode = 0;      // ограничение на 3 режима
    }
    // ----------------------отработка нажатия SET и изменение режимов---------------------------

    // ------------------режим ВАРИВОЛЬТ-------------------
    if (mode == 0 && !vape_state && !set_hold) {
      if (mode_flag) {                     // приветствие
        mode_flag = 0;
          display.clearDisplay();               
          display.setTextSize(3);               
          display.setTextColor(WHITE);          
          display.setCursor(0,10);             
          display.println("VAR VOL");             
          display.display(); 
        delay(400);
        display.clearDisplay();
      }
      //---------кнопка ВВЕРХ--------
      if (up_state && !up_flag) {
        volts += 100;
        volts = min(volts, bat_volt_f);  // ограничение сверху на текущий заряд акума
        up_flag = 1;
        display.clearDisplay();
      }
      if (!up_state && up_flag) {
        up_flag = 0;
        change_v_flag = 1;
      }
      //---------кнопка ВВЕРХ--------

      //---------кнопка ВНИЗ--------
      if (down_state && !down_flag) {
        volts -= 100;
        volts = max(volts, 0);
        down_flag = 1;
        display.clearDisplay();
      }
      if (!down_state && down_flag) {
        down_flag = 0;
        change_v_flag = 1;
      }
      //---------кнопка ВНИЗ--------
      display.clearDisplay();              
      display.setTextSize(3);              
      display.setTextColor(WHITE);         
      display.setCursor(0,0);            
      display.println(String((float)volts / 1000));            
      display.display();  
    }
    // ------------------режим ВАРИВОЛЬТ-------------------


    // ------------------режим ВАРИВАТТ-------------------
    if (mode == 1 && !vape_state && !set_hold) {
      if (mode_flag) {                     
        mode_flag = 0;
        display.clearDisplay();               
        display.setTextSize(3);              
        display.setTextColor(WHITE);          
        display.setCursor(0,10);             
        display.println("VAR VAT");             
        display.display(); 
        delay(400);
        display.clearDisplay(); 
      }
      //---------кнопка ВВЕРХ--------
      if (up_state && !up_flag) {
        watts += 1;
        byte maxW = (sq((float)bat_volt_f / 1000)) / ohms;
        watts = min(watts, maxW);               // ограничение сверху на текущий заряд акума и сопротивление
        up_flag = 1;
        display.clearDisplay();
      }
      if (!up_state && up_flag) {
        up_flag = 0;
        change_w_flag = 1;
      }
      //---------кнопка ВВЕРХ--------

      //---------кнопка ВНИЗ--------
      if (down_state && !down_flag) {
        watts -= 1;
        watts = max(watts, 0);
        down_flag = 1;
        display.clearDisplay();
      }
      if (!down_state && down_flag) {
        down_flag = 0;
        change_w_flag = 1;
      }
      //---------кнопка ВНИЗ--------
      display.clearDisplay();              
          display.setTextSize(3);              
          display.setTextColor(WHITE);         
          display.setCursor(0,10);            
          display.println(String(watts));             
          display.display(); 
    }
    // ------------------режим ВАРИВАТТ--------------

    // ----------режим установки сопротивления-----------
    if (mode == 2 && !vape_state && !set_hold) {
      if (mode_flag) {                    
        mode_flag = 0;
        display.clearDisplay();              
          display.setTextSize(3);              
          display.setTextColor(WHITE);          
          display.setCursor(0,10);             
          display.println("COIL");            
          display.display(); 
        delay(400);
        display.clearDisplay();  
      }
      //---------кнопка ВВЕРХ--------
      if (up_state && !up_flag) {
        ohms += 0.05;
        ohms = min(ohms, 3);
        up_flag = 1;
        //disp.clear();
        display.clearDisplay();  
      }
      if (!up_state && up_flag) {
        up_flag = 0;
        change_o_flag = 1;
      }
      //---------кнопка ВВЕРХ--------

      //---------кнопка ВНИЗ--------
      if (down_state && !down_flag) {
        ohms -= 0.05;
        ohms = max(ohms, 0);
        down_flag = 1;
        display.clearDisplay();  
      }
      if (!down_state && down_flag) {
        down_flag = 0;
        change_o_flag = 1;
      }
      //---------кнопка ВНИЗ--------
      display.clearDisplay();              
          display.setTextSize(3);              
          display.setTextColor(WHITE);          
          display.setCursor(0,0);            
          display.println(String(ohms));             
          display.display(); 
    }
    // ----------режим установки сопротивления-----------

    //---------отработка нажатия кнопки парения-----------
    if (vape_state && flag && !wake_up_flag) {

      if (!vape_flag) {
        vape_flag = 1;
        vape_mode = 1;            // первичное нажатие
        delay(20);                // анти дребезг (сделал по-тупому, лень)
        vape_press = millis();    // первичное нажатие
      }

      if (vape_release_count == 1) {
        vape_mode = 2;               // двойное нажатие
        delay(20);                   // анти дребезг (сделал по-тупому, лень)
      }
      if (vape_release_count == 2) {
        vape_mode = 3;               // тройное нажатие
      }

      if (millis() - vape_press > vape_threshold * 1000) {  // "таймер затяжки"
        vape_mode = 0;
        digitalWrite(mosfet, 0);
      }

      if (vape_mode == 1) {                                           // обычный режим парения

         display.clearDisplay();              
          display.setTextSize(2);              
          display.setTextColor(WHITE);         
          display.setCursor(0,0);            
          display.println("NORMAL");           
          display.display(); 
          display.setCursor(0,15);  
          display.println("MODE");            
          display.display();
        if (mode == 0) {                                              // если ВАРИВОЛЬТ
          PWM = (float)volts / bat_volt_f * 1024;                     // считаем значение для ШИМ сигнала
          if (PWM > 1023) PWM = 1023;                                 // ограничил PWM "по тупому", потому что constrain сука не работает!
          PWM_f = PWM_filter_k * PWM + (1 - PWM_filter_k) * PWM_old;  // фильтруем
          PWM_old = PWM_f;                                            // фильтруем
        }
        Timer1.pwm(mosfet, PWM_f);                                    // управление мосфетом
      }
      if (vape_mode == 2 && turbo_mode) {                             // турбо режим парения (если включен)
        
          display.clearDisplay();             
          display.setTextSize(2);              
          display.setTextColor(WHITE);         
          display.setCursor(0,0);            
          display.println("TURBO");            
          display.display(); 
          display.setCursor(0,15);  
          display.println("MODE");             
          display.display();
        digitalWrite(mosfet, 1);                                      // херачить на полную мощность
      }
      if (vape_mode == 3) {                                           // тройное нажатие
        vape_release_count = 0;
        vape_mode = 1;
        vape_flag = 0;
        good_night();    // вызвать функцию сна
      }
      vape_btt = 1;
    }

    if (!vape_state && vape_btt) {  // если кнопка ПАРИТЬ отпущена
      if (millis() - vape_press > 180) {
        vape_release_count = 0;
        vape_mode = 0;
        vape_flag = 0;
      }
      vape_btt = 0;
      if (vape_mode == 1) {
        vape_release_count = 1;
        vape_press = millis();
      }
      if (vape_mode == 2) vape_release_count = 2;

      digitalWrite(mosfet, 0);
      display.clearDisplay();
      mode_flag = 0;

      // если есть изменения в настройках, записать в память
      if (change_v_flag) {
        EEPROM.writeInt(0, volts);
        change_v_flag = 0;
      }
      if (change_w_flag) {
        EEPROM.writeInt(2, watts);
        change_w_flag = 0;
      }
      if (change_o_flag) {
        EEPROM.writeFloat(4, ohms);
        change_o_flag = 0;
      }
      // если есть изменения в настройках, записать в память
    }
    if (vape_state && !flag) { // если акум сел, а мы хотим подымить
      display.clearDisplay();               
    display.setTextSize(3);               
    display.setTextColor(WHITE);         
    display.setCursor(0,10);            
    display.println("BAT LOW");             
    display.display();  
      vape_flag = 1;
    }
    //---------отработка нажатия кнопки парения-----------
  }

  if (wake_up_flag) wake_puzzle();                   // вызвать функцию 5 нажатий для пробудки

  if (millis() - wake_timer > sleep_timer * 1000) {  // если кнопки не нажимались дольше чем sleep_timer секунд
    good_night();
  }
} // конец loop

//------функция, вызываемая при выходе из сна (прерывание)------
void wake_up() {
  digitalWrite(disp_vcc, HIGH);  // включить дисплей
  Timer1.disablePwm(mosfet);    // принудительно отключить койл
  digitalWrite(mosfet, LOW);    // принудительно отключить койл
  wake_timer = millis();         // запомнить время пробуждения
  wake_up_flag = 1;
  vape_release_count = 0;
  vape_mode = 0;
  vape_flag = 0;
  mode_flag = 1;
}
//------функция, вызываемая при выходе из сна (прерывание)------

//------функция 5 нажатий для полного пробуждения------
void wake_puzzle() {
  detachInterrupt(0);    // отключить прерывание
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Запуск дисплея
  delay(50);
  vape_btt_f = 0;
  boolean wake_status = 0;
  byte click_count = 0;
  while (1) {
    vape_state = !digitalRead(butt_vape);

    if (vape_state && !vape_btt_f) {
      vape_btt_f = 1;
      click_count++;
      switch (click_count) {
        case 1: 
        display.clearDisplay();              
        display.setTextSize(3);               
        display.setTextColor(WHITE);         
        display.setCursor(0,0);           
        display.println("1");             
        display.display();      
          break;
        case 2: 
        display.setCursor(15,0);            
        display.println("2");            
        display.display(); 
          break;
        case 3: 
        display.setCursor(30,0);             
        display.println("3");            
        display.display(); 
          break;
        case 4: 
        display.setCursor(45,0);             
        display.println("4");             
        display.display(); 
          break;
      }
      if (click_count > 4) {               // если 5 нажатий сделаны за 3 секунды
        wake_status = 1;                   // флаг "проснуться"
        break;
      }
    }
    if (!vape_state && vape_btt_f) {
      vape_btt_f = 0;
      delay(70);
    }
    if (millis() - wake_timer > 5000) {    // если 5 нажатий не сделаны за 3 секунды
      wake_status = 0;                     // флаг "спать"
      break;
    }
  }
  if (wake_status) {
    wake_up_flag = 0;
    display.clearDisplay();
  } else {
    display.clearDisplay();
    good_night();     // спать
  }
}
//------функция 5 нажатий для полного пробуждения------

//-------------функция ухода в сон----------------
void good_night() {
  display.clearDisplay();               //Очистить дисплей
        display.setTextSize(2);               //Задаем размер текста
        display.setTextColor(WHITE);          //Задаем цвет текста
        display.setCursor(0,0);             //Задаем координату начала текста в пикселях
        display.println("GOOD");             //Пишем сам текст  
        display.display();
        display.setCursor(0,15);  
        display.println("NIGHT");             //Пишем сам текст  
        display.display();
  delay(500);
  display.clearDisplay();
  Timer1.disablePwm(mosfet);    // принудительно отключить койл
  digitalWrite(mosfet, LOW);    // принудительно отключить койл
  delay(50);  
  digitalWrite(disp_vcc, LOW);    // подать 0 на все пины питания дисплея
  delay(50);  
  attachInterrupt(0, wake_up, FALLING);   // подключить прерывание для пробуждения 
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);    // спать. mode POWER_OFF, АЦП выкл
}
//-------------функция ухода в сон----------------

//----------режим теста кнопок----------
void service_mode() {
  if (set_state && !set_flag) {
    set_flag = 1;
    Serial.println("SET pressed");
  }
  if (!set_state && set_flag) {
    set_flag = 0;
    Serial.println("SET released");
  }
  if (up_state && !up_flag) {
    up_flag = 1;
    Serial.println("UP pressed");
  }
  if (!up_state && up_flag) {
    up_flag = 0;
    Serial.println("UP released");
  }
  if (down_state && !down_flag) {
    down_flag = 1;
    Serial.println("DOWN pressed");
  }
  if (!down_state && down_flag) {
    down_flag = 0;
    Serial.println("DOWN released");
  }
  if (vape_state && !vape_flag) {
    vape_flag = 1;
    Serial.println("VAPE pressed");
  }
  if (!vape_state && vape_flag) {
    vape_flag = 0;
    Serial.println("VAPE released");
  }
}
//----------режим теста кнопок----------

void timerIsr()  //нужно для дисплея
{
 // disp.timerIsr();
}

void calibration() {
  //--------калибровка----------
  for (byte i = 0; i < 7; i++) EEPROM.writeInt(i, 0);          // чистим EEPROM для своих нужд
  my_vcc_const = 1.1;
  Serial.print("Real VCC is: "); Serial.println(readVcc());     // общаемся с пользователем
  Serial.println("Write your VCC (in millivolts)");
  while (Serial.available() == 0); int Vcc = Serial.parseInt(); // напряжение от пользователя
  float real_const = (float)1.1 * Vcc / readVcc();              // расчёт константы
  Serial.print("New voltage constant: "); Serial.println(real_const, 3);
  EEPROM.writeFloat(8, real_const);                             // запись в EEPROM
  while (1); // уйти в бесконечный цикл
  //------конец калибровки-------
}

long readVcc() { //функция чтения внутреннего опорного напряжения, универсальная (для всех ардуин)
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;

  result = my_vcc_const * 1023 * 1000 / result; // расчёт реального VCC
  return result; // возвращает VCC
}
