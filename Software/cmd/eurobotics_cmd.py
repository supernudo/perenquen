#! /usr/bin/env python

import os,sys,atexit
import termios
import serial
from select import select
import cmd
#import pylab
from  matplotlib import pylab
from math import *

import re
import numpy as np
import matplotlib.pyplot as plt

import struct
import numpy
import shlex
import time
import math
import warnings
warnings.filterwarnings("ignore","tempnam",RuntimeWarning, __name__)

import logging
log = logging.getLogger("EuroboticsShell")
_handler = logging.StreamHandler()
_handler.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
log.addHandler(_handler)
log.setLevel(1)

EUROBOTICS_PATH=os.path.dirname(sys.argv[0])

SPM_PAGE_SIZE = 256
METADATA_ADDR = 256

def crc_ccitt_update (crc, data):
    """crc argument is the previous value of 16 bits crc (the initial
    value is 0xffff). 'data' is the 8 bits value added to crc. The
    function returns the new crc value."""

    data ^= (crc & 0xff)
    data ^= (data << 4)
    data &= 0xff

    ret = (data << 8) & 0xffff
    ret |= ((crc >> 8) & 0xff)
    ret ^= ((data >> 4) & 0xff)
    ret ^= ((data << 3) & 0xffff)
    return ret

def do_crc(buf):
    i = 0
    crc = 0xffff
    sum = 0
    while i < len(buf):
        crc = crc_ccitt_update(crc, ord(buf[i]))
        sum +=  ord(buf[i])
        i += 1
    return (crc << 16) + (sum & 0xffff)

def prog_page(ser, addr, buf):
    """program a page from buf at addr"""

    # switch in program mode
    ser.flushInput()
    ser.write('p')

    # send address
    s = ser.readline()
    if not s.endswith("addr?\r\n"):
        print "failed (don't match addr)"
        return -1
    ser.write("%x\n"%addr)
    s = ser.readline()
    if not s.startswith("ok"):
        print "failed"
        return -1

    # fill page with buf data
    page = [ '\xff' ] * SPM_PAGE_SIZE
    i = 0
    while i < SPM_PAGE_SIZE and i < len(buf):
        page[i] = buf[i]
        i += 1

    # send data
    i = 0
    while i < SPM_PAGE_SIZE:
        c = page[i]
        ser.write(c)
        i += 1

    sys.stdout.write(".")
    sys.stdout.flush()

    # compare crc
    avr_crc = int(ser.readline()[0:8], 16)

    crc = do_crc(page)
    if crc != avr_crc:
        print "failed: bad crc %x %x"%(crc, avr_crc)
        ser.write('n')
        return -1

    ser.write('y')
    s = ser.readline()
    if not s.startswith("OK"):
        print "failed"
        return -1
    return 0

def prog_metadata(ser, addr, buf):
    length = len(buf)
    crc = do_crc(buf)
    page = struct.pack(">LL", length, crc)
    filename = os.path.join(EUROBOTICS_PATH, "binaries/%x_%x.bin"%(length, crc))
    print "saving in %s"%(filename)
    f = open(filename, "w")
    f.write(buf)
    f.close()
    return prog_page(ser, addr, page)

def get_same_bin_file(ser):
    l = read32(ser, METADATA_ADDR)
    c = read32(ser, METADATA_ADDR + 4)
    filename = os.path.join(EUROBOTICS_PATH,
                            "binaries/%x_%x.bin"%(l, c))
    print filename
    try:
        f = open(filename, "r")
    except:
        return None
    buf = f.read()
    f.close()
    print "found old bin matching <%s>"%(filename)
    return buf

def read32(ser, addr):
    """read a 32 bits value at addr"""

    ser.write('\n')
    ser.write('\n')
    ser.write('\n')
    time.sleep(0.2)
    ser.flushInput()

    # switch in program mode
    ser.write('d')

    # send address
    time.sleep(0.1)

    s = ser.readline()
    print repr(s)
    if not s.endswith("addr?\r\n"):
        print "failed (don't match addr)"
        return -1
    print addr
    ser.write("%x\n"%addr)
    s = ser.readline()
    print repr(s)
    return int(s, 16)

def check_crc(ser, buf, offset, size):
    """Process the crc of buf, ask for a crc of the flash, and check
    that value is correct"""
    if size <= 0:
        return 0

    # go in CRC mode
    ser.flushInput()
    ser.write('c')

    # send addr
    s = ser.readline()
    if not s.endswith("addr?\r\n"):
        print "failed <%s>"%s
        return -1
    ser.write("%x\n"%offset)

    # send size
    s = ser.readline()
    if not s.startswith("size?"):
        print "failed"
        return -1
    ser.write("%x\n"%size)

    # compare CRC
    crc = do_crc(buf[offset:offset+size])
    avr_crc = int(ser.readline()[0:8], 16)
    if crc != avr_crc:
        return -1
    return 0


class SerialLogger:
    def __init__(self, ser, filein, fileout=None):
        self.ser = ser
        self.filein = filein
        self.fin = open(filein, "a", 0)
        if fileout:
            self.fileout = fileout
            self.fout = open(fileout, "a", 0)
        else:
            self.fileout = filein
            self.fout = self.fin
    def fileno(self):
        return self.ser.fileno()
    def read(self, *args):
        res = self.ser.read(*args)
        self.fin.write(res)
        return res
    def write(self, s):
        self.fout.write(s)
        self.ser.write(s)

class Interp(cmd.Cmd):
    prompt = "Eurobotics > "
    def __init__(self, tty, baudrate=921600):
        cmd.Cmd.__init__(self)
        self.ser = serial.Serial(tty,baudrate=baudrate)
        self.escape  = "\x01" # C-a
        self.quitraw = "\x02" # C-b
        self.serial_logging = False
        self.default_in_log_file = "/tmp/eurobotics.in.log"
        self.default_out_log_file = "/tmp/eurobotics.out.log"

    def do_quit(self, args):
        return True

    def do_log(self, args):
        """Activate serial logs.
        log <filename>           logs input and output to <filename>
        log <filein> <fileout>   logs input to <filein> and output to <fileout>
        log                      logs to /tmp/eurobotics.log or the last used file"""

        if self.serial_logging:
            log.error("Already logging to %s and %s" % (self.ser.filein,
                                                        self.ser.fileout))
        else:
            self.serial_logging = True
            files = [os.path.expanduser(x) for x in args.split()]
            if len(files) == 0:
                files = [self.default_in_log_file, self.default_out_log_file]
            elif len(files) == 1:
                self.default_in_log_file = files[0]
                self.default_out_log_file = None
            elif len(files) == 2:
                self.default_in_log_file = files[0]
                self.default_out_log_file = files[1]
            else:
                print "Can't parse arguments"

            self.ser = SerialLogger(self.ser, *files)
            log.info("Starting serial logging to %s and %s" % (self.ser.filein,
                                                               self.ser.fileout))


    def do_unlog(self, args):
        if self.serial_logging:
            log.info("Stopping serial logging to %s and %s" % (self.ser.filein,
                                                               self.ser.fileout))
            self.ser = self.ser.ser
            self.serial_logging = False
        else:
            log.error("No log to stop")


    def do_raw(self, args):
        "Switch to RAW mode"
        stdin = os.open("/dev/stdin",os.O_RDONLY)
        stdout = os.open("/dev/stdout",os.O_WRONLY)

        stdin_termios = termios.tcgetattr(stdin)
        raw_termios = stdin_termios[:]

        try:
            log.info("Switching to RAW mode")

            # iflag
            raw_termios[0] &= ~(termios.IGNBRK | termios.BRKINT |
                                termios.PARMRK | termios.ISTRIP |
                                termios.INLCR | termios.IGNCR |
                                termios.ICRNL | termios.IXON)
            # oflag
            raw_termios[1] &= ~termios.OPOST;
            # cflag
            raw_termios[2] &= ~(termios.CSIZE | termios.PARENB);
            raw_termios[2] |= termios.CS8;
            # lflag
            raw_termios[3] &= ~(termios.ECHO | termios.ECHONL |
                                termios.ICANON | termios.ISIG |
                                termios.IEXTEN);

            termios.tcsetattr(stdin, termios.TCSADRAIN, raw_termios)

            mode = "normal"
            while True:
                ins,outs,errs=select([stdin,self.ser],[],[])
                for x in ins:
                    if x == stdin:
                        c = os.read(stdin,1)
                        if mode  == "escape":
                            mode =="normal"
                            if c == self.escape:
                                self.ser.write(self.escape)
                            elif c == self.quitraw:
                                return
                            else:
                                self.ser.write(self.escape)
                                self.ser.write(c)
                        else:
                            if c == self.escape:
                                mode = "escape"
                            else:
                                self.ser.write(c)
                    elif x == self.ser:
                        os.write(stdout,self.ser.read())
        finally:
            termios.tcsetattr(stdin, termios.TCSADRAIN, stdin_termios)
            log.info("Back to normal mode")

    def bootloader(self, filename, boardnum):
        self.ser.write("")
        time.sleep(0.4)
        self.ser.write("bootloader\n")
        time.sleep(0.4)
        self.ser.write("\n")

        print "start programming"
        self.ser.flushInput()
        f = open(filename)
        buf = f.read()
        f.close()

        addr = 0

        old_buf = get_same_bin_file(self.ser)
        #old_buf = None

        while addr < len(buf):
            if addr > METADATA_ADDR and old_buf != None and \
                    old_buf[addr:addr+SPM_PAGE_SIZE] == buf[addr:addr+SPM_PAGE_SIZE]:
                sys.stdout.write("-")
                sys.stdout.flush()
                addr += SPM_PAGE_SIZE
                continue
            time.sleep(0.1)
            if check_crc(self.ser, buf, addr, SPM_PAGE_SIZE) == 0:
                sys.stdout.write("*")
                sys.stdout.flush()
            elif prog_page(self.ser, addr,
                         buf[addr:addr+SPM_PAGE_SIZE]) != 0:
                return
            addr += SPM_PAGE_SIZE
        if check_crc(self.ser, buf, 0, len(buf)):
            print "crc failed"
            return
        print
        if prog_metadata(self.ser, METADATA_ADDR, buf) != 0:
            print "metadata failed"
            return
        print "Done."
        self.ser.write("x")
        self.do_raw("")

    def do_bootloader(self, args):
        self.bootloader(args, 0)

    def do_mainboard(self, args):
        filename = os.path.join(MICROB_PATH, "../mainboard/main.bin")
        self.bootloader(filename, 1)

    def do_cobboard(self, args):
        filename = os.path.join(MICROB_PATH, "../cobboard/main.bin")
        self.bootloader(filename, 2)

    def do_ballboard(self, args):
        filename = os.path.join(MICROB_PATH, "../ballboard/main.bin")
        self.bootloader(filename, 3)

    def do_centrifugal(self, args):
        try:
            sa, sd, aa, ad = [int(x) for x in shlex.shlex(args)]
        except:
            print "args: speed_a, speed_d, acc_a, acc_d"
            return
        print sa, sd, aa, ad
        time.sleep(10)
        self.ser.write("traj_speed angle %d\n"%(sa))
        time.sleep(0.1)
        self.ser.write("traj_speed distance %d\n"%(sd))
        time.sleep(0.1)
        self.ser.write("traj_acc angle %d\n"%(aa))
        time.sleep(0.1)
        self.ser.write("traj_acc distance %d\n"%(ad))
        time.sleep(0.1)
        self.ser.write("goto da_rel 800 180\n")
        time.sleep(3)
        self.ser.flushInput()
        self.ser.write("position show\n")
        time.sleep(1)
        print self.ser.read()

    def do_toto(self, args):
        print args
        time.sleep(1)
        self.ser.write("gain angle 20 0 0\n")

    def do_traj_acc(self, args):
        try:
            name, acc = [x for x in shlex.shlex(args)]
        except:
            print "args: cs_name acc"
            return

        acc = int(acc)
        self.ser.write("quadramp %s %d %d 0 0\n"%(name, acc, acc))
        time.sleep(1)
        print self.ser.read()

    def do_traj_speed(self, args):
        try:
            name, speed = [x for x in shlex.shlex(args)]
        except:
            print "args: cs_name speed"
            return

        speed = int(speed)
        self.ser.write("traj_speed %s %d\n"%(name, speed))
        time.sleep(1)
        print self.ser.read()

    def do_tune(self, args):
        try:
            name, tlog, cons, gain_p, gain_i, gain_d = [x for x in shlex.shlex(args)]
        except:
            print "args: cs_name, time_ms, consigne, gain_p, gain_i, gain_d"
            return

        tlog = float(tlog)/1000.0
        cons = int(cons)
        gain_p = int(gain_p)
        gain_i = int(gain_i)
        gain_d = int(gain_d)
        print name, cons, gain_p, gain_i, gain_d

        self.ser.write("position set 1500 1000 0\n")
        print("position set 1500 1000 0\n")
        time.sleep(1)

        # cs log on
        self.ser.write("log type cs on\n")
        print("log type cs on\n")
        time.sleep(0.1)

        if name == "d":
            self.ser.write("gain distance %d %d %d\n"%(gain_p, gain_i, gain_d))
            self.ser.write("goto d_rel %d\n"%(cons))
        elif name == "a":
            self.ser.write("gain angle %d %d %d\n"%(gain_p, gain_i, gain_d))
            self.ser.write("goto a_rel %d\n"%(cons))
        else:
            print "unknow cs name"
            return

        # time print
        time.sleep(0.01)
        t1 = time.time()

        TS = 5
        i = 0
        t = np.zeros(0)
        us = np.zeros(0)
        cons = f_cons = err = feedback = out = np.zeros(0)
        v_cons = v_feedback = np.zeros(1)
        a_cons = a_feedback = a_cons = a_feedback = np.zeros(2)

        while True:

          # cs log off
          t2 = time.time()
          if tlog and (t2 - t1) >= tlog:
            tlog = 0
            self.ser.write("\n")
            self.ser.write("log type cs off\n")
            #print (t2 - t1)

          # read log data
          time.sleep(TS/10000.0)
          line = self.ser.readline()

          tm_data = struct.struct('Iiiiiiiiiiic')

          print(line)
          #m = re.match("(-?\+?\d+).(-?\+?\d+): \((-?\+?\d+),(-?\+?\d+),(-?\+?\d+)\) "
          #             "%s cons= (-?\+?\d+) fcons= (-?\+?\d+) err= (-?\+?\d+) "
          #             "in= (-?\+?\d+) out= (-?\+?\d+)"%(name), line)

          m = re.match("%s (\d+) (-?\+?\d+) (-?\+?\d+) (-?\+?\d+) (-?\+?\d+) (-?\+?\d+)"%(name), line)


          # data logging
          if m:
            #print line
            #print m.groups()
            t = np.append(t, i*TS)
            us = np.append(us, int(m.groups()[0]))
            cons = np.append(cons, int(m.groups()[1]))
            f_cons = np.append(f_cons, int(m.groups()[2]))
            err = np.append(err, int(m.groups()[3]))
            feedback = np.append(feedback, int(m.groups()[4]))
            out = np.append(out, int(m.groups()[5]))

            if i>0:
                v_cons = np.append(v_cons, (f_cons[i] - f_cons[i-1])/(us[i]-us[i-1]))
                v_feedback = np.append(v_feedback, (feedback[i] - feedback[i-1])/(us[i]-us[i-1]))

            if i>1:
                a_cons = np.append(a_cons, (v_cons[i] - v_cons[i-1])/(us[i]-us[i-1]))
                a_feedback = np.append(a_feedback, (v_feedback[i] - v_feedback[i-1])/(us[i]-us[i-1]))

            i += 1
            continue

          # trajectory end
          m = re.match("returned", line)
          if m:
            print line.rstrip()
            us = us - us[0]

            plt.figure(1)
            plt.subplot(311)
            plt.plot(us,v_cons,'.-', label="consigna")
            plt.plot(us,v_feedback,'.-', label="feedback")
            plt.ylabel('v (pulsos/Ts)')
            plt.grid(True)
            plt.legend()
            plt.title('%s kp=%s, ki=%d, kd=%d'%(name, gain_p, gain_i, gain_d))

            plt.subplot(312)
            plt.plot(us,a_cons,'.-', label="consigna")
            plt.plot(us,a_feedback,'.-', label="feedback")
            plt.ylabel('a (pulsos/Ts^2)')
            plt.grid(True)
            plt.legend()

            plt.subplot(313)
            plt.plot(us,out,'.-')
            plt.xlabel('t (ms)')
            plt.ylabel('u (cuentas)')
            plt.grid(True)


            plt.figure(2)
            plt.subplot(311)
            plt.plot(us,f_cons-feedback[0], '.-', label="consigna")
            plt.plot(us,feedback-feedback[0], '.-', label="feedback")
            plt.ylabel('posicion (pulsos)')
            plt.grid(True)
            plt.legend()
            plt.title('%s kp=%s, ki=%d, kd=%d'%(name, gain_p, gain_i, gain_d))

            plt.subplot(312)
            plt.plot(us,err, '.-')
            plt.xlabel('t (ms)')
            plt.ylabel('error (pulsos)')
            plt.grid(True)


            plt.subplot(313)
            plt.plot(us,out,'.-')
            plt.xlabel('t (ms)')
            plt.ylabel('u (cuentas)')
            plt.grid(True)

            plt.show()
            break

        # print unknow strigs
        print line.rstrip()

if __name__ == "__main__":
    try:
        import readline,atexit
    except ImportError:
        pass
    else:
        histfile = os.path.join(os.environ["HOME"], ".eurobotics_history")
        atexit.register(readline.write_history_file, histfile)
        try:
            readline.read_history_file(histfile)
        except IOError:
            pass

    device = "COM4"
    #device = "/dev/rfcomm0"
    if len(sys.argv) > 1:
        device = sys.argv[1]
    interp = Interp(device)
    while 1:
        try:
            interp.cmdloop()
        except KeyboardInterrupt:
            print
        except Exception,e:
            l = str(e).strip()
            if l:
                log.exception("%s" % l.splitlines()[-1])
            continue
        break
