'''
 Licence: GPLv3
 Copyright 2021 Muthuswamy Pugazhendi <muthuswamy.pugazhendi@gmail.com>
 Name:- M.Pugazhendi
 Date:-  12thJul2021
 Version:- V0.1
 e-mail:- muthuswamy.pugazhendi@gmail.com
'''
import utime

class LCD16x2:
    
    def __init__(self, RS, ENABLE, BACK_LIGHT, D4, D5, D6, D7):
        self.RS = RS
        self.ENABLE = ENABLE
        self.BACK_LIGHT = BACK_LIGHT
        self.D4 = D4
        self.D5 = D5
        self.D6 = D6
        self.D7 = D7
        self.Reset()
    
    def Reset(self):
        self.RS.value(0)
        self.WriteCommand(0x03)
        self.WriteCommand(0x03)
        self.WriteCommand(0x03)
        
        #Initialize LCD into 4 bit mode
        self.WriteCommand(0x02)
        
        #Enable 5x7 character mode
        self.WriteCommand(0x28)
        
        #Cursor off
        self.WriteCommand(0x0C)
        
        #Increment cursor
        self.WriteCommand(0x06)
        
        #Clear screen
        self.WriteCommand(0x01)
        
        #Sleep for two mSeconds
        utime.sleep_ms(2)
     
    # Generate EnablePulse
    def EnablePulse(self):
        self.ENABLE.value(1)
        utime.sleep_us(40)
        self.ENABLE.value(0)
        utime.sleep_us(40)

    # Write a byte to LCD
    # Separate into 2 nibbles and then write to LCD
    def WriteByte(self, data):
       self.D4.value((data & 0b00010000) >>4)
       self.D5.value((data & 0b00100000) >>5)
       self.D6.value((data & 0b01000000) >>6)
       self.D7.value((data & 0b10000000) >>7)
       self.EnablePulse()
       
       self.D4.value((data & 0b00000001) >>0)
       self.D5.value((data & 0b00000010) >>1)
       self.D6.value((data & 0b00000100) >>2)
       self.D7.value((data & 0b00001000) >>3)
       self.EnablePulse()
       
    # Write a command to LCD
    def WriteCommand(self, data):
        # Disable Register Select
        self.RS.value(0)
        # Write Command Byte to LCD
        self.WriteByte(data)
        
    # Write a data to LCD
    def WriteData(self, data):
        # Enable Register Select
        self.RS.value(1)
        # Write Command Byte to LCD
        self.WriteByte(data)
        # Disable Register Select
        self.RS.value(0)
    
    # Writes a string into Line 1 or Line2
    def WriteLine(self, string, line_number):
        if(line_number == 1):
            self.WriteCommand(0x80)
            for x in string:
                self.WriteData(ord(x))
        if(line_number == 2):
            self.WriteCommand(0xC0)
            for x in string:
                self.WriteData(ord(x))    
    # Clear Screen
    def ClearScreenCursorHome(self):
        self.WriteCommand(0x01)
        self.WriteCommand(0x02)
        # Clear screen and put the cursor into Home needs longer time
        # Introduce two mSeconds delay
        utime.sleep_ms(2)
        
    # Back light On
    def BackLightOn(self):
        self.BACK_LIGHT.value(1)
        
    # Back light Off
    def BackLightOff(self):
        self.BACK_LIGHT.value(0)
        
    # Cursor On
    def CursorOn(self):
        self.WriteCommand(0x0E)

    # Cursor Blinking
    def CursorBlink(self):
        self.WriteCommand(0x0D)
        
    # Cursor Off
    def CursorOff(self):
        self.WriteCommand(0x0C)
