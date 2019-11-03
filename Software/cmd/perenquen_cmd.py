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

from bluetooth import (
    BluetoothSocket,
    RFCOMM,
)
from bluetooth.btcommon import BluetoothError

from struct import unpack

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
    prompt = "balitronics > "
    def __init__(self, tty=None, baudrate=38400, addr='98:D3:31:70:13:AB', port=1):
        cmd.Cmd.__init__(self)
        self.escape  = "\x01" # C-a
        self.quitraw = "\x02" # C-b
        self.serial_logging = False
        self.default_in_log_file = "/tmp/eurobotics.in.log"
        self.default_out_log_file = "/tmp/eurobotics.out.log"

        self.ser = None
        if (tty is not None):
            self.ser = serial.Serial(tty,baudrate=baudrate)
        else:
            self.rfcomm = BluetoothSocket(RFCOMM)
            self.rfcomm.connect((addr, port))
            self.rfcomm.settimeout(0.01)

    def do_quit(self, args):
        if self.ser is None:
            self.rfcomm.close()
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
                print("Can't parse arguments")

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


    def do_raw_tty(self, args):
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
                        os.write(stdout,self.ser.read(1))
        finally:
            termios.tcsetattr(stdin, termios.TCSADRAIN, stdin_termios)
            log.info("Back to normal mode")

    def do_tm_rfcomm(self, args):
        self.rfcomm.settimeout(1.)
        self.rfcomm.send("event power off\n\r")
        time.sleep(0.1)
        self.rfcomm.send("event tm on\n\r")

        t1 = t2 = time.time()
        self.rfcomm.settimeout(0.01)
        received = self.rfcomm.recv(1)
        while t2 - t1 < 1:
            try:
                self.rfcomm.settimeout(0.01)
                #received.append(self.rfcomm.recv(1024))
                received += self.rfcomm.recv(1024)
            except BluetoothError as error:
                if str(error) != 'timed out':
                    raise
            t2 = time.time()

        self.rfcomm.send("event tm on\n\r")
        time.sleep(0.1)

        print(received)

    def do_tm_test(self, args):
        self.rfcomm.settimeout(1.)
        self.rfcomm.send("\n\revent tm on\n\r")
        self.rfcomm.settimeout(0.1)
        #self.rfcomm.settimeout(None) # set blocking True
        #self.rfcomm.settimeout(0.0) # set blocking False
        time.sleep(5)
        t1 = time.time()
        while time.time() - t1 < 10:
            try:
                received = self.rfcomm.recv(4096)
                print("rx", len(received))
                received = 0
            #except BluetoothError as error:
            except:
                raise
                #if str(error) != 'timed out':
                #    raise
                #if str(error) != 'Resource temporarily unavailable':
                #    raise

    def do_raw_rfcomm(self, args):
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
            #i = 0

            mode = "normal"
            while True:
                #ins,outs,errs=select([stdin,self.ser],[],[])
                ins,outs,errs=select([stdin,self.rfcomm],[],[])
                for x in ins:
                    if x == stdin:
                        c = os.read(stdin,1)
                        if mode  == "escape":
                            mode =="normal"
                            if c == self.escape:
                                #self.ser.write(self.escape)
                                self.rfcomm.settimeout(1.)
                                self.rfcomm.send(self.escape)
                            elif c == self.quitraw:
                                return
                            else:
                                #self.ser.write(self.escape)
                                #self.ser.write(c)
                                self.rfcomm.settimeout(1.)
                                self.rfcomm.send(self.escape)
                        else:
                            if c == self.escape:
                                mode = "escape"
                            else:
                                #time.sleep(0.1)
                                #self.ser.write(c)
                                self.rfcomm.settimeout(1.)
                                self.rfcomm.send(c)

                    #elif x == self.ser:
                    elif x == self.rfcomm:
                        #os.write(stdout,self.ser.read())
                        #self.rfcomm.settimeout(0.01)
                        #os.write(stdout,self.rfcomm.recv(1022))

                        try:
                            self.rfcomm.settimeout(0.0) # set blocking False
                            received = self.rfcomm.recv(4096)
                            os.write(stdout,received)
                        except:
                            pass
                        #except BluetoothError as error:
                        #    if str(error) != 'timed out':
                        #        raise


        finally:
            termios.tcsetattr(stdin, termios.TCSADRAIN, stdin_termios)
            log.info("Back to normal mode")


    def do_centrifugal(self, args):
        try:
            sa, sd, aa, ad = [int(x) for x in shlex.shlex(args)]
        except:
            print("args: speed_a, speed_d, acc_a, acc_d")
            return
        print(sa, sd, aa, ad)
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
        print(self.ser.read())

    def do_traj_acc(self, args):
        try:
            name, acc = [x for x in shlex.shlex(args)]
        except:
            print("args: cs_name acc")
            return

        acc = int(acc)
        self.ser.flushInput()
        self.ser.write("quadramp %s %d %d 0 0\n"%(name, acc, acc))
        time.sleep(1)
        print(self.ser.read())

    def do_traj_speed(self, args):
        try:
            name, speed = [x for x in shlex.shlex(args)]
        except:
            print("args: cs_name speed")
            return

        speed = int(speed)
        self.ser.flushInput()
        self.ser.write("traj_speed %s %d\n"%(name, speed))
        time.sleep(1)
        print(self.ser.read())

    def do_hc05_stress(self, args):
        #self.ser.flushInput()
        #self.ser.flushInput()
        for i in range(1000):
            for j in range(9):
                self.rfcomm.settimeout(1.)
                self.rfcomm.send(str(j))
                #print(str(i)+'\n'))

        data = ''
        for i in range(1000):
            for j in range(9):
                data += self.rfcomm.recv(1)

        for i in range(1000):
            for j in range(9):
                print(data[i])

    def do_tune(self, args):
        try:
            name, tlog, cons, gain_p, gain_i, gain_d = [x for x in shlex.shlex(args)]
        except:
            print("args: cs_name, time_ms, consigne, gain_p, gain_i, gain_d")
            return

        # Test parameters
        tlog = float(tlog)/1000.0
        cons = int(cons)
        gain_p = int(gain_p)
        gain_i = int(gain_i)
        gain_d = int(gain_d)
        #print(name, cons, gain_p, gain_i, gain_d)


        # Set position, gains, set tm on and goto consign
        self.ser.flushInput()
        self.ser.write("position set 1500 1000 0\n")
        time.sleep(0.1)

        if name == "d":
            self.ser.write("gain distance %d %d %d\n"%(gain_p, gain_i, gain_d))
            time.sleep(0.1)
            self.ser.write("echo off\n")
            #time.sleep(2)
            #print(self.ser.read())

            print("goto d_rel %d\n"%(cons))

            self.ser.write("event tm on\n")
            self.ser.flushInput()
            time.sleep(0.1)
            self.ser.write("goto d_rel %d\n"%(cons))

        elif name == "a":
            self.ser.write("gain angle %d %d %d\n"%(gain_p, gain_i, gain_d))
            time.sleep(0.1)
            self.ser.write("echo off\n")
            #time.sleep(2)
            #print(self.ser.read())

            print("goto d_rel %d\n"%(cons))

            self.ser.write("event tm on\n")
            self.ser.flushInput()
            time.sleep(0.1)
            self.ser.write("goto a_rel %d\n"%(cons))

        else:
            print("unknow cs name")
            return

        # Telemetry reading
        tm_data = ''
        t1 = time.time()
        self.ser.flushInput()
        while True:
          tm_data += self.ser.read()

          # Break after test time
          t2 = time.time()
          if tlog and (t2 - t1) >= tlog:
            tlog = 0
            self.ser.write("\n\r")
            time.sleep(0.1)
            self.ser.write("event tm off\n")
            self.ser.write("echo on\n")
            time.sleep(2)
            self.ser.flushInput()
            time.sleep(2)
            self.ser.flushInput()
            break


        # Telemetry parser
        i = 0
        time_ms = np.zeros(0)
        cons = f_cons = err = feedback = out = np.zeros(0)
        v_cons = v_feedback = np.zeros(1)
        a_cons = a_feedback = a_cons = a_feedback = np.zeros(2)

        TM_HEAD_BYTE_0 = '\xFB'
        TM_HEAD_BYTE_1 = '\xBF'
        TM_TAIL_BYTE_0 = '\xED'
        TM_SIZE = 48
        TM_STRUCT = 'IiiiiiiiiiiBB'

        head = TM_HEAD_BYTE_0 + TM_HEAD_BYTE_1
        tm_packets = tm_data.split(head)
        #print(tm_packets))

        for data in tm_packets:
            if data[-1] == TM_TAIL_BYTE_0 and len(data) == (TM_SIZE-2):
                tm_data = unpack(TM_STRUCT, data)
                #print(tm_data, type(tm_data)))

                time_ms = np.append(time_ms, tm_data[0])
                #cons = np.append(cons, tm_data[1])
                #f_cons = np.append(f_cons, tm_data[2])
                #err = np.append(err, tm_data[3])
                #feedback = np.append(feedback, tm_data[4])
                #out = np.append(out, tm_data[5])
                cons = np.append(cons, tm_data[6])
                f_cons = np.append(f_cons, tm_data[7])
                err = np.append(err, tm_data[8])
                feedback = np.append(feedback, tm_data[9])
                out = np.append(out, tm_data[10])


                if i>0:
                    v_cons = np.append(v_cons, (f_cons[i] - f_cons[i-1])/(time_ms[i]-time_ms[i-1]))
                    v_feedback = np.append(v_feedback, (feedback[i] - feedback[i-1])/(time_ms[i]-time_ms[i-1]))

                if i>1:
                    a_cons = np.append(a_cons, (v_cons[i] - v_cons[i-1])/(time_ms[i]-time_ms[i-1]))
                    a_feedback = np.append(a_feedback, (v_feedback[i] - v_feedback[i-1])/(time_ms[i]-time_ms[i-1]))

                i += 1


        # Telemetry stuff

        """
        # Telemetry parsing
        t1 = time.time()
        while True:

          # Break after test time
          t2 = time.time()
          if tlog and (t2 - t1) >= tlog:
            tlog = 0
            self.ser.write("event tm off\n")
            break


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
            #print(line)
            #print(m.groups())
            t = np.append(t, i*TS)
            time_ms = np.append(time_ms, int(m.groups()[0]))
            cons = np.append(cons, int(m.groups()[1]))
            f_cons = np.append(f_cons, int(m.groups()[2]))
            err = np.append(err, int(m.groups()[3]))
            feedback = np.append(feedback, int(m.groups()[4]))
            out = np.append(out, int(m.groups()[5]))

            if i>0:
                v_cons = np.append(v_cons, (f_cons[i] - f_cons[i-1])/(time_ms[i]-time_ms[i-1]))
                v_feedback = np.append(v_feedback, (feedback[i] - feedback[i-1])/(time_ms[i]-time_ms[i-1]))

            if i>1:
                a_cons = np.append(a_cons, (v_cons[i] - v_cons[i-1])/(time_ms[i]-time_ms[i-1]))
                a_feedback = np.append(a_feedback, (v_feedback[i] - v_feedback[i-1])/(time_ms[i]-time_ms[i-1]))

            i += 1
            continue

          # trajectory end
          m = re.match("returned", line)
          if m:
            print(line.rstrip())

         """

        time_ms = time_ms - time_ms[0]

        plt.figure(1)
        plt.subplot(311)
        plt.plot(time_ms,v_cons,'.-', label="consigna")
        plt.plot(time_ms,v_feedback,'.-', label="feedback")
        plt.ylabel('v (pulsos/Ts)')
        plt.grid(True)
        plt.legend()
        plt.title('%s kp=%s, ki=%d, kd=%d'%(name, gain_p, gain_i, gain_d))

        plt.subplot(312)
        plt.plot(time_ms,a_cons,'.-', label="consigna")
        plt.plot(time_ms,a_feedback,'.-', label="feedback")
        plt.ylabel('a (pulsos/Ts^2)')
        plt.grid(True)
        plt.legend()

        plt.subplot(313)
        plt.plot(time_ms,out,'.-')
        plt.xlabel('t (ms)')
        plt.ylabel('u (cuentas)')
        plt.grid(True)


        plt.figure(2)
        plt.subplot(311)
        plt.plot(time_ms,f_cons-feedback[0], '.-', label="consigna")
        plt.plot(time_ms,feedback-feedback[0], '.-', label="feedback")
        plt.ylabel('posicion (pulsos)')
        plt.grid(True)
        plt.legend()
        plt.title('%s kp=%s, ki=%d, kd=%d'%(name, gain_p, gain_i, gain_d))

        plt.subplot(312)
        plt.plot(time_ms,err, '.-')
        plt.xlabel('t (ms)')
        plt.ylabel('error (pulsos)')
        plt.grid(True)


        plt.subplot(313)
        plt.plot(time_ms,out,'.-')
        plt.xlabel('t (ms)')
        plt.ylabel('u (cuentas)')
        plt.grid(True)

        plt.show()



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

    device = None
    if len(sys.argv) > 1:
        device = sys.argv[1]
    interp = Interp(device) # HC-05 blue
    #interp = Interp(device, addr='98:D3:31:F6:1C:51') # HC-05 white
    while 1:
        try:
            interp.cmdloop()
        except KeyboardInterrupt:
            print
        except Exception as e:
            l = str(e).strip()
            if l:
                log.exception("%s" % l.splitlines()[-1])
            continue
        break
