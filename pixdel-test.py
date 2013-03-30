#test pixdel
# Jacques Deschênes
# 2013-03-29
#
import serial
import time
import argparse
import msvcrt


device=''
done = False


parser=argparse.ArgumentParser(description='test RTS et DTR control line on serial port',
                               usage='module-test.py [-h]  device-name ')
parser.add_argument('serial', help='serial port name')
cmd_line=parser.parse_args()
device=cmd_line.serial


def sync():
    term.write(chr(0xFF))

def cmd(id,r,g,b):
    sync()
    term.write(chr(id)+chr(r)+chr(g)+chr(b))
	
term=serial.Serial(port=device,baudrate=14400,timeout=0)

delay=.005
pixdel_id=1
print 'test 1, controle intensite blanc'
while not done:
    for i in range(255):
        r=g=b=i
        cmd(pixdel_id,r,g,b)
        time.sleep(delay)
        if msvcrt.kbhit(): 
            done=True
            break
			
    for i in range(255):
        r=g=b=254-i
        cmd(pixdel_id,r,g,b)
        time.sleep(delay)
        if msvcrt.kbhit(): 
            done=True
            break;
			
c=msvcrt.getch()
done=False
print 'test 2, scintillement'
cmd(0,0x01,0x01,0x01)
while not done:
    if msvcrt.kbhit(): done=True;
    time.sleep(.001)

	
c=msvcrt.getch()
done=False
print 'test 3, cycle rouge,vert,bleu'
while not done:
    cmd(pixdel_id,0x01,0,0)
    time.sleep(.25)
    cmd(pixdel_id,0,0x01,0)
    time.sleep(.25)
    cmd(pixdel_id,0,0,0x01)
    time.sleep(.25)
    if msvcrt.kbhit(): done=True;    
	
c=msvcrt.getch()
done=False
print 'test 4, controle intensitee une seule couleur'
while not done:
    for i in range(128):
        r=i
        cmd(pixdel_id,r,0,0)
        time.sleep(.01)
        if msvcrt.kbhit(): 
            done=True
            break
			
    for i in range(128):
        r=127-i
        cmd(pixdel_id,r,0,0)
        time.sleep(.01)
        if msvcrt.kbhit(): 
            done=True
            break;

c=msvcrt.getch()
done=False
print 'test 5, stabilite pwm lorsque le bus est sature de commandes'
cmd(0,0x01,0x01,0x01)
while not done:
    cmd(2,2,3,4)
    if msvcrt.kbhit(): done=True;

			
	
cmd(pixdel_id,0,0,0)
term.close() 

