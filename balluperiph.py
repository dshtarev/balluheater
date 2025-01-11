from collections import deque
import neopixel
from machine import Pin,ADC,Timer

class BalluKeys:
    Up         = 1
    Down       = 2
    OpenWindow = 3
    Automatic  = 4
    Power      = 5
    Manual     = 6

def linear_interpolation(x, points):
    # Сортируем точки по x
    points = sorted(points)

    # Проверяем, что x находится в пределах значений x в точках
    if x < points[0][0] or x > points[-1][0]:
        raise ValueError("x находится вне диапазона заданных точек.")
    # Находим два соседних значения
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        if x0 <= x <= x1:
            # Линейная интерполяция
            y = y0 + (y1 - y0) * (x - x0) / (x1 - x0)
            return y
    # Если не нашли подходящие точки, возвращаем None (это не должно произойти)
    return None


class ADCFilter:
    def __init__(self, window_size):
        """
        Инициализация фильтра.
        :param window_size: Количество точек для сглаживания.
        """
        self.window_size = window_size
        self.values = deque((),window_size)  # Очередь фиксированной длины

    def filter(self, new_value):
        """
        Фильтрация нового значения и возврат сглаженного значения.
        :param new_value: Новое значение отсчета от АЦП.
        :return: Сглаженное значение.
        """
        self.values.append(new_value)  # Добавляем новое значение в очередь
        smoothed_value = sum(self.values) / len(self.values)  # Вычисляем среднее
        return smoothed_value



class TemperatureSensor:
    t_sensor_calib_data = [(2000, 0.7), (1329, 15.9), (987, 26.9), (977, 27.4),(755, 35.3),(697, 37.4),(613, 41.2),(400, 51.8),(202, 68.0),(160, 71.0)]

    def __init__(self, _adc ):
        self.adc = _adc;
        self.adcfilter = ADCFilter(10);
        self.lastvalue = 0;

    def get_temperature(self):
        return  self.lastvalue;

    def do_conversion(self):
        t = linear_interpolation( self.adc.read(), self.t_sensor_calib_data );
        self.lastvalue = self.adcfilter.filter( t );

class Keyboard:
    def __init__(self,_i2c, _irqpin, _callback ):
        #XW09 INTERRUPT
        self.callback = _callback
        self.i2c = _i2c;
        self.irqpin = _irqpin;
        self.irqpin.irq(trigger=Pin.IRQ_FALLING, handler=self.key_pressed_callback)

    def key_pressed_callback(self,m):
        print('pin change', self.irqpin)
        button =self.i2c.readfrom(0x40,2);
        print( hex( button[0] ) + " " + hex(button[1] ) );

        if button[0] == 0xff and button[1] == 0xdf :
            print("up button");
            self.callback( BalluKeys.Up );

        if button[0] == 0xff and button[1] == 0xbf:
            print("down button");
            self.callback( BalluKeys.Down );

        if button[0] == 0xf7 and button[1] == 0xff:
            print("dow button");
            self.callback( BalluKeys.OpenWindow );

        if button[0] == 0xfe and button[1] == 0xff:
            print("a button");
            self.callback( BalluKeys.Automatic );

        if button[0] == 0xfb and button[1] == 0xff:
            print("pwr button");
            self.callback( BalluKeys.Power );

        if button[0] == 0xfd and button[1] == 0xff:
            self.callback( BalluKeys.Manual );

class GN1668:
    def __init__(self,clkPin, dioPin, stbPin ):
        self.clk = clkPin;
        self.stb = stbPin;
        self.dio = dioPin;

    def write(self,byte, stbCtrl = True):
        if( stbCtrl == True ):
            self.stb.off();
        for i in range(0,8):
            b = ( byte >> i ) & 0x1;
            #print(str(i) + "-" + str(b) );
            self.clk.off();
            if(b == 1):
                self.dio.on()
            else:
                self.dio.off();
            #time.sleep_us(10);
            self.clk.on();
            #time.sleep_us(10);

        if( stbCtrl ):
            self.stb.on();

        self.clk.off();

    def memWrite( self, mem, intensity, on=True ):
        self.stb.on();
        self.clk.off();
        self.dio.off();

        #gn1668_write(0x80 );  # DISPLAY OFF
        self.write(0x40 + 0x04*0 );   #WRITE DATA TO DISPLAY

        self.stb.off();
        self.write(0xC0  , stbCtrl=False);

        for i in range(0,0x0E):
            self.write( mem[i] , stbCtrl=False);
            #time.sleep_us(100);

        self.stb.on();

        self.write(0x00);    # set display mode
        self.write(0x80 + 0x8*on + ( intensity&0x7 ));  #switch display on

class BalluDisplay:
    digit1 = [0x42,0x52,0x13,0x33,0x03,0x62,0x72,0x0];
    digit2 = [0x44,0x54,0x15,0x35,0x05,0x64,0x74,0x0];
    digDecoder = {" ":0x00, "0": 0x3F,"1": 0x06, "2": 0x5B, "3":0x4F, "4":0x66, "5":0x6D,"6":0x7D,"7":0x7,"8":0x7F,"9":0x6F,"A":0x77,"B":0x7C,
                  "C":0x39,"d":0x5E,"D":0x5E,"E": 0x79, "F":0x71, "[":0x39 , "]":0x0F, "-":0x40,"_":0x08 }

    powerBarCoord = [0x37,0x17,0x07,0x76,0x66,0x56,0x46,0x36,0x26,0x16];

    def __init__(self,_gn1668,_unk):
        self.gn1668 = _gn1668;
        self.intensity  = 0x7;
        self.wifiFlag   = False;
        self.powerLevel = 0x0;
        self.AFlag      = False;
        self.MFlag      = False;
        self.digIndicator = "  ";
        self.powerFLag = False;
        self.openWindowFlag = False;
        self.timeFlag = False;
        self.scheduleFlag = False;
        self.gn1668_displaymem = bytearray(0xE);
        self.n = neopixel.NeoPixel(_unk,2,3);
        self.led1 = (0,0,0);
        self.led2 = (0,0,0);
        self.displayOn = False;

    def  renderPowerBar(self,level):
        for i in range(0,10):
            coord = self.powerBarCoord[i];
            bitIdx = ( coord >> 4 )&0xF;
            byteIdx = coord & 0xF;

            lBit = 0
            if( level > i ):
                lBit = 1;

            if lBit == 1:
                self.gn1668_displaymem[byteIdx] |= (1<<bitIdx);
            else:
                self.gn1668_displaymem[byteIdx] &= 0xFF ^ (1<<bitIdx)

    def render_digit(self,num,d):
        digCode = self.digDecoder[ d ];
        for i in range(0,7):
            digBit = (digCode >> i) & 0x1;
            if num == 0:
                coord = self.digit1[i];
            else:
                coord = self.digit2[i];
            bitIdx = ( coord >> 4 )&0xF;
            byteIdx = coord & 0xF;

            if digBit == 1:
                self.gn1668_displaymem[byteIdx] |= (1<<bitIdx);
            else:
                self.gn1668_displaymem[byteIdx] &= 0xFF ^ (1<<bitIdx)

    def set_display_on( self, on ):
        self.displayOn = on;

    def render_display(self):
        self.render_digit( 0, self.digIndicator[0] );
        self.render_digit( 1, self.digIndicator[1] );

        if( self.wifiFlag == True ):
            self.gn1668_displaymem[ 1 ] |= 0x1;
        else:
            self.gn1668_displaymem[ 1 ] &= (0xFF ^ 0x1);


        if( self.AFlag == True ):
            self.gn1668_displaymem[ 0 ] |= 1 << 6;
        else:
            self.gn1668_displaymem[ 0 ] &= (0xFF ^ ( 1 << 6 ) );

        if( self.MFlag == True ):
            self.gn1668_displaymem[ 1 ] |= 1 << 1;
        else:
            self.gn1668_displaymem[ 1 ] &= (0xFF ^ ( 1 << 1 ) );


        self.renderPowerBar ( self.powerLevel );

    def led_update(self):
        print("led update");
        self.n[0] = self.led1;
        self.n[1] = self.led2;
        self.n.write();

    def hide_led(self):
        print("Hide led");
        self.n[0] = (0,0,0);
        self.n[1] = (0,0,0);
        self.n.write();
        self.n.write();

    def update(self):
        if( self.displayOn == False ):
            self.fill_display_memory(0x0);
            self.hide_led();
        else:
            self.render_display();
            self.led_update();

        self.gn1668.memWrite(self.gn1668_displaymem, self.intensity, self.displayOn );

    def fill_display_memory( self, b ):
        for i in range(0, 0xE ):
            self.gn1668_displaymem[i] = b;


#heater controller

class BalluHeaterController:
    def __init__( self , r1p, r2p, tp, powerLevel = 0 ):
        self.r1Pin = r1p;
        self.r2Pin = r2p;
        self.triacPin = tp;
        self.powerLevel = powerLevel;
        self.pwmCnt = 0

        tim0 = Timer(1);
        tim0.init(mode=Timer.PERIODIC, freq=100, callback=self.generate_pwm )

    def off(self):
        sel.set_power( 0 );

    def set_power(self, level ):
        print("Set heater power level:" + str(level) );
        self.powerLevel = level;
        if( level <= 0 ):
            self.r1Pin.off();
            self.r2Pin.off();
            self.triacPin.off();
        if( level >= 100 ):
            self.r1Pin.on(); #on
            self.r2Pin.on(); #on
            self.triacPin.on(); #off
        if( level >= 50 and level < 100 ):
            self.r1Pin.on();    #on
            self.r2Pin.off();
        if( level >0 and level < 50 ):
            self.r1Pin.off();
            self.r2Pin.off();

    def generate_pwm( self , id ):

        pl = self.powerLevel;
        if( pl > 100 ):
            pl = 100;
        if( pl < 0 ):
            pl = 0;

        duty = pl % 50;   #50
        self.pwmCnt += 1

        if( (self.pwmCnt%100) < (duty) ):    #duty*2
            self.triacPin.on();
        else:
            self.triacPin.off();


