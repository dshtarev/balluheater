import time
from machine import Pin,PWM,ADC,Timer
from umqttsimple import MQTTClient
import ubinascii
import micropython
import network
#import esp
import socket
import machine
import _thread
import wifimgr
from pid import PID
import esp32
import gc
import os
from balluperiph import ADCFilter
from balluperiph import TemperatureSensor
from balluperiph import Keyboard
from balluperiph import GN1668
from balluperiph import BalluDisplay
from balluperiph import BalluKeys
from balluperiph import BalluHeaterController
import json

#webpage
def web_page():
    gpio_state="OFF"

    wlan_sta = wifimgr.wlan_sta

    html = """<html><head> <title>ESP Web Server</title> <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,"> <style>html{font-family: Helvetica; display:inline-block; margin: 0px auto; text-align: center;}
    h1{color: #0F3376; padding: 2vh;}p{font-size: 1.5rem;}.button{display: inline-block; background-color: #e7bd3b; border: none;
    border-radius: 4px; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
    .button2{background-color: #4286f4;}</style></head><body> <h1>ESP Web Server</h1>
    <p>GPIO state: <strong>""" + gpio_state + """</strong></p><p><a href="/?led=on"><button class="button">ON</button></a></p>
    <p><a href="/?led=off"><button class="button button2">OFF</button></a></p><p>""" + wlan_sta.config('mac').hex() + """</body></html>"""
    return html


def doWeb(s):
    #time.sleep_ms(1000);
    conn, addr = s.accept()
    print('Got a connection from %s' % str(addr))
    request = conn.recv(1024)
    request = str(request)
    print('Content = %s' % request)
    led_on = request.find('/?led=on')
    led_off = request.find('/?led=off')
    if led_on == 6:
        print('LED ON')
        triac.on();
    if led_off == 6:
        print('LED OFF')
        triac.off();
    response = web_page()
    conn.send('HTTP/1.1 200 OK\n')
    conn.send('Content-Type: text/html\n')
    conn.send('Connection: close\n\n')
    conn.sendall(response)
    conn.close()

def web_handler_thread_function(threadname,tid ):
    #webpart
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 80))
    s.listen(5)

    while True:
        doWeb(s);


def wifi_handler_thread_function(threadname,tid ):
    while True:
        balluHeater.setWifiConnected( False );
        wlan = wifimgr.get_connection()
        if wlan is None:
            print("Could not initialize the network connection.")
            time.sleep(1);
        else:
            balluHeater.setWifiConnected( True );
            balluHeater.update_display();
            while( wlan.isconnected() ):
                time.sleep(1);

#mqtt part
def sub_cb(topic, msg):
    print((topic, msg))

def mqtt_create_client():
  global client_id, mqtt_server
  client = MQTTClient(client_id, mqtt_server, user=mqtt_user, password=mqtt_pass)
  client.set_callback(sub_cb)
  return client

def restart_and_reconnect():
  print('Failed to connect to MQTT broker. Reconnecting...')
  time.sleep(10)
  #machine.reset()



#hardware part
cntx = 0;

def tim0_callback(id):
    global cntx
    cntx = cntx + 1;

    if( (cntx % 10) == 0 ):
        if( balluHeater.heaterMode == BalluHeaterMode.manual ):
            balluHeater.set_power_level( balluHeater.powerLevelManual );
        else:
            sig = balluHeater.pid.update( ts1.get_temperature(), balluHeater.temperatureSetpoint );
            if( balluHeater.pid.integration > 50 ):
                balluHeater.pid.integration = 50 ;
            if( balluHeater.pid.integration < 0 ):
                balluHeater.pid.integration = 0 ;
            balluHeater.set_power_level( sig );

        balluHeater.set_temperature_current( ts1.get_temperature() );

    ts1.do_conversion();
    balluHeater.do_every_10ms();
    balluHeater.update_display();

def beep():
    pwm.init();
    time.sleep_ms(100);
    pwm.deinit();




#main class

class BalluHeaterMode:
    manual = 0
    automatic = 1

#
class DisplayState:
    TemperatureCurrent = 0
    TemperatureSetpoint = 1

class BalluHeater:
        maxPowerLevel = 100
        showTemperatureSetpointDuration = 5
        configFileName = "config.json"

        def __init__(self):

            self.displayState = DisplayState.TemperatureCurrent;
            self.displayStateTimer = 0;

            self.pid = PID(5,2,0.1);
            self.pid.ki_enable( True );

            scl = Pin(32);
            sda = Pin(33);
            self.i2c = machine.SoftI2C(scl,sda );

            irqpin= Pin(26, Pin.IN);
            self.keyboard = Keyboard(self.i2c,irqpin, self.handle_button_press );

            #power
            r1pin = Pin(4, Pin.OUT);
            r2pin = Pin(21, Pin.OUT);
            r3pin = Pin(22, Pin.OUT);
            triacPin  = Pin(23, Pin.OUT);
            self.heaterController = BalluHeaterController(r1pin,r2pin,triacPin );

            self.heaterMode = BalluHeaterMode.manual;
            self.powerLevelManual = 0                  #Текущая мощность в ручном режим
            self.powerLevelAutomatic = 0
            self.temperatureSetpoint = 28               #Уставка температуры
            self.temperatureSetpointRange = (3,50);
            self.temperatureCurrent = 0;
            self.powerOn = True;
            self.updateDisplayFlag = True;
            self.wifiConnectedFlag = False;
            self.powerLimit = 100;

            self.load_config();
            gc.collect();

            self.set_power_limit( self.powerLimit );

            balluDisplay.set_display_on( True );

        def load_config(self):
            if( self.configFileName in os.listdir() ):
                print("Load config...");
                configFile = open(self.configFileName,"r")
                cd = json.loads( configFile.read() );
                configFile.close();

                self.heaterMode = cd["heaterMode"];
                self.powerLevelManual = cd["powerLevelManual"];
                self.temperatureSetpoint = cd["temperatureSetpoint"];
                self.powerLimit = cd["powerLimit"];

            else:
                print("Config not found... Creating a new one...");
                self.set_defaults();
                self.save_config();

        def set_defaults(self):
            self.heaterMode = BalluHeaterMode.manual;
            self.powerLevelManual = 0                  #Текущая мощность в ручном режим
            self.temperatureSetpoint = 28               #Уставка температуры
            self.powerLimit = 100;

        def save_config(self):
            cf = open( self.configFileName, "w+" );
            cd = {};
            cd["heaterMode"] = int(self.heaterMode);
            cd["powerLevelManual"] = self.powerLevelManual;
            cd["temperatureSetpoint"] = self.temperatureSetpoint;
            cd["powerLimit"] = self.powerLimit;
            outJson = json.dumps( cd );
            cf.write( outJson );
            cf.close();

        def inc_temperature_setpoint(self):

            if( self.displayState == DisplayState.TemperatureCurrent ):
                self.displayState = DisplayState.TemperatureSetpoint
            else:
                self.temperatureSetpoint += 1;
                minT,maxT = self.temperatureSetpointRange
                if( self.temperatureSetpoint >  maxT ):
                    self.temperatureSetpoint = maxT

            self.updateDisplayFlag = True;

        def dec_temperature_setpoint(self):
            if( self.displayState == DisplayState.TemperatureCurrent ):
                self.displayState = DisplayState.TemperatureSetpoint
            else:
                self.temperatureSetpoint -= 1;
                minT,maxT = self.temperatureSetpointRange

                if( self.temperatureSetpoint <  minT ):
                    self.temperatureSetpoint = minT

            self.updateDisplayFlag = True;

        def set_temperature_current(self, t ):
            if( int(t) != int(self.temperatureCurrent) ):
                self.updateDisplayFlag = True;

            self.temperatureCurrent = t;

        def set_temperature_setpoint(self,sp):
            self.temperatureSetpoint = sp;
            if( self.temperatureSetpoint >  maxT ):
                self.temperatureSetpoint = maxT

            if( self.temperatureSetpoint <  minT ):
                self.temperatureSetpoint = minT

            self.updateDisplayFlag = True;

        def inc_power_level_manual(self):
            self.powerLevelManual+=10;
            if( self.powerLevelManual >  BalluHeater.maxPowerLevel ):
                self.powerLevelManual = 0
            self.set_power_level( self.powerLevelManual  );
            self.updateDisplayFlag = True;

        def set_power_level( self, level ):
            clampLevel = level
            if( self.heaterMode == BalluHeaterMode.manual ):
                self.powerLevelManual   = clampLevel;
            else:
                if( clampLevel >  self.powerLimit ):
                    clampLevel = self.powerLimit;
                self.powerLevelAutomatic = clampLevel;

            self.heaterController.set_power( clampLevel );
            self.updateDisplayFlag = True;

        def set_heater_mode(self,mode):
            self.heaterMode = mode;
            self.updateDisplayFlag = True;

        def set_power_on(self,pwr):
            print("Set power on - " + str( pwr ) );
            self.powerOn = pwr;
            balluDisplay.set_display_on( pwr );
            self.updateDisplayFlag = True;
            if( pwr == True ):
                machine.reset();
            else:
                self.set_power_level( 0 );
                self.save_config();

        def setWifiConnected(self, flag ):
                self.wifiConnectedFlag = flag;
                self.updateDisplayFlag = True;


        def update_display( self ):
            global balluDisplay;

            if( self.updateDisplayFlag == False ):
                return;

            print("Update dislay invoked...");

            if self.heaterMode == BalluHeaterMode.automatic:
                balluDisplay.AFlag = True;
                balluDisplay.MFlag = False;
            else:
                balluDisplay.AFlag = False;
                balluDisplay.MFlag = True;

            balluDisplay.wifiFlag = self.wifiConnectedFlag;

            if( self.heaterMode == BalluHeaterMode.manual ):
                balluDisplay.powerLevel = self.powerLevelManual/10;
            else:
                balluDisplay.powerLevel = self.powerLevelAutomatic/10;

            tempSp = 0;
            if( self.displayState == DisplayState.TemperatureCurrent ):
                tempSp = int(self.temperatureCurrent);
            else:
                tempSp = int(self.temperatureSetpoint);

            balluDisplay.digIndicator = f"{tempSp:02d}";

            balluDisplay.update();
            self.updateDisplayFlag = False;

        def do_every_10ms(self):
            if( self.displayState == DisplayState.TemperatureSetpoint ):
                self.displayStateTimer += 1;

            if( self.displayStateTimer > self.showTemperatureSetpointDuration*10 ):
                self.displayStateTimer = 0;
                self.displayState = DisplayState.TemperatureCurrent ;

        def set_power_limit(self, limit ):
            self.powerLimit = limit;

            if( self.powerLimit == 50 ):
                balluDisplay.led1 = ( 0,0,255 );
            else:
                balluDisplay.led1 = ( 255,0,0 );

        def handle_button_press(self, button ):
            if button == BalluKeys.Up:
                print("up button");
                self.inc_temperature_setpoint();

            if button == BalluKeys.Down:
                print("down button");
                self.dec_temperature_setpoint();

            if button == BalluKeys.OpenWindow:
                print("dow button");

            if button == BalluKeys.Automatic:
                print("a button");

                if( self.heaterMode == BalluHeaterMode.automatic ):
                    self.powerLimit = 50 if self.powerLimit == 100 else 100

                self.set_power_limit( self.powerLimit );

                print("Power Limit: " + str( self.powerLimit ) );

                self.set_heater_mode( BalluHeaterMode.automatic );

            if button == BalluKeys.Power:
                print("pwr button");
                self.set_power_on( not self.powerOn )

            if button == BalluKeys.Manual:
                if( self.heaterMode == BalluHeaterMode.manual ):
                    self.inc_power_level_manual();
                else:
                    self.set_heater_mode ( BalluHeaterMode.manual );

            beep();


gc.enable();

#TIMER
tim0 = Timer(0);
# periodic at 1kHz
tim0.init(mode=Timer.PERIODIC, freq=10, callback=tim0_callback )


#BEEP
pwm = PWM(Pin(18), freq=440, duty=512) # create and configure in one go
pwm.deinit();

#TEMP SENSOR
adc = ADC(Pin(39) );
adc.atten(ADC.ATTN_11DB);
ts1 = TemperatureSensor( adc );


#gn1668
dio = Pin(13, Pin.OUT);
clk = Pin(14, Pin.OUT);
stb = Pin(27, Pin.OUT);
unk = Pin(25, Pin.OUT);

gn1668 = GN1668( clk,dio,stb );

#display
balluDisplay = BalluDisplay(gn1668, unk);
balluDisplay.intensity = 7;
balluDisplay.fill_display_memory( 0x00 );
balluDisplay.update();

balluDisplay.led1 = (255,0,0);
balluDisplay.led2 = (255,255,255);
balluDisplay.led_update();

#mqtt
client_id ="sdfasdfaf"
#mqtt_server="mq.kincony.com"
mqtt_server="192.168.0.224"
mqtt_user="dshtarev@gmail.com"
mqtt_pass="Qaplzxmn13"
last_message = 0
message_interval = 1
counter = 0

#_thread.start_new_thread(web_handler_thread_function, (0,0))
_thread.start_new_thread(wifi_handler_thread_function, (0,0))

mqtt_client = mqtt_create_client()
mqtt_connected = False

#main class
balluHeater = BalluHeater();
balluHeater.update_display();
#-----------

mac = wifimgr.wlan_sta.config('mac').hex()
mqtt_topic_prefix = "BALLU" + mac;

while True:
  try:
    if( mqtt_connected ):
        mqtt_client.check_msg()
        if (time.time() - last_message) >= message_interval:
            temp = ts1.get_temperature();

            tf = esp32.raw_temperature()
            tc = (tf-32.0)/1.8

            params={}
            params["cnt"] = counter;
            params["tc"]  = temp;
            params["tsp"] = balluHeater.temperatureSetpoint;
            params["cpl"] = balluHeater.heaterController.powerLevel;
            params["intt"] = tc;
            params["plim"] = balluHeater.powerLimit;
            params["mode"] = "manual" if balluHeater.heaterMode == BalluHeaterMode.manual else "automatic";

            payload = json.dumps(params);
            print("Publis msg counter:" + str( payload ) );
            mqtt_client.publish(mqtt_topic_prefix+"/params",  payload  )

            print("Mem alloc" + str( gc.mem_alloc() ) );

            last_message = time.time()
            counter += 1

    if( wifimgr.wlan_sta.isconnected() ):
        if( mqtt_connected == False ):
            try:
                print("MQTT trying to connect")
                mqtt_client.connect()
                mqtt_connected = True;
                print("Mqtt connected!");
            except OSError as e:
                print("MQTT connection failed")
                time.sleep(1)
                mqtt_connected = False;


    #doWeb();

  except OSError as e:
        print("Exception while mqtt connect");
        time.sleep(1)
        restart_and_reconnect()






