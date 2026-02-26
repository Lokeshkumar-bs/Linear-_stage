import sys, time, struct, threading
import serial, serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QLineEdit, QGroupBox, QGridLayout, QTextEdit, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QTextCursor, QIntValidator, QDoubleValidator


PULSE_DIV     = 1      # SAP 154 — confirmed at this setting
CURRENT       = 150    # Motor current (hardcoded)
ACCEL         = 500    # Acceleration (hardcoded)
UNITS_PER_CMS = 396.9  # calibrated: 446.5 * (10/11.25)
MAX_VEL_UNITS = 2047
HOME_THRESHOLD = 50    # GAP 1 counts — stop return when position <= this (~0.015 cm)

def cmm_to_units(cm_min):
    """cm/min → TMC429 velocity units (0-2047)"""
    cm_s = float(cm_min) / 60.0
    return max(1, min(int(cm_s * UNITS_PER_CMS), MAX_VEL_UNITS))

def travel_time(dist_cm, speed_cmm):
    """Seconds to travel dist_cm at speed in cm/min"""
    cm_s = float(speed_cmm) / 60.0
    return float(dist_cm) / cm_s if cm_s > 0 else 0.0


# ── TMCL Driver ──────────────────────────────────────────────────
class TMCL:
    def __init__(self, port, address=1):
        self.address = address
        self._lock = threading.Lock()
        self.ser = serial.Serial(port, baudrate=9600, timeout=0.5)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, instruction, type_, motor, value):
        with self._lock:
            value = value & 0xFFFFFFFF
            cmd = [self.address, instruction, type_, motor,
                   (value>>24)&0xFF, (value>>16)&0xFF,
                   (value>>8)&0xFF, value&0xFF]
            cmd.append(sum(cmd) & 0xFF)
            self.ser.reset_input_buffer()
            self.ser.write(bytes(cmd))
            time.sleep(0.025)
            reply = self.ser.read(9)
            if len(reply) == 9:
                return struct.unpack('>i', reply[4:8])[0]
            return None

    def ror(self, s):     self.send(1, 0, 0, int(s))
    def rol(self, s):     self.send(2, 0, 0, int(s))
    def mst(self):
        for _ in range(3):
            self.send(3, 0, 0, 0)
            time.sleep(0.02)
    def sap(self, p, v):  self.send(5, p, 0, int(v))
    def gap(self, p):     return self.send(6, p, 0, 0)
    def stop_r(self):     return self.gap(10)   # right limit (STOP_R) — safety only
    def stop_l(self):     return self.gap(11)   # left limit  (STOP_L) — safety only
    def zero_pos(self):   self.sap(1, 0)        # zero position counter at current location
    def get_pos(self):    return self.gap(1)     # read position counter (counts)


# ── Signals ──────────────────────────────────────────────────────
class Sig(QObject):
    log    = pyqtSignal(str)
    status = pyqtSignal(str, str)
    done   = pyqtSignal()
    cycles = pyqtSignal(int)


# ── Stylesheet ───────────────────────────────────────────────────
STYLE = """
QMainWindow, QWidget {
    background: #f5f5f5;
    color: #111111;
    font-family: Arial, sans-serif;
    font-size: 14px;
}
QGroupBox {
    background: #ffffff;
    border: 1px solid #dddddd;
    border-radius: 6px;
    margin-top: 10px;
    padding: 10px 12px 10px;
    font-size: 16px;
    font-weight: bold;
    color: #000000;
    letter-spacing: 1px;
    font-family: Arial;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    background: #f5f5f5;
    color: #000000;
}
QPushButton {
    background: #ffffff;
    color: #005533;
    border: 2px solid #005533;
    border-radius: 5px;
    padding: 7px 14px;
    font-family: Arial;
    font-weight: bold;
    font-size: 12px;
    min-width: 80px;
}
QPushButton:hover    { background: #e6f2ed; }
QPushButton:pressed  { background: #ccebde; }
QPushButton:disabled { color: #cccccc; border-color: #cccccc; background: #f8f8f8; }
QPushButton#stop     { color: #bb0000; border-color: #bb0000; }
QPushButton#stop:hover    { background: #fff0f0; }
QPushButton#stop:disabled { color: #cccccc; border-color: #cccccc; }
QPushButton#sm   { padding: 5px 10px; font-size: 11px; min-width: 50px; }
QPushButton#dis  { color: #bb0000; border-color: #bb0000; padding: 5px 10px; font-size: 11px; min-width: 90px; }
QPushButton#dis:hover { background: #fff0f0; }
QPushButton#jog  { padding: 10px 16px; font-size: 13px; font-weight: bold;
                   min-width: 140px; min-height: 44px; }
QPushButton#home { background: #005533; color: #ffffff; border: 2px solid #005533;
                   padding: 10px 16px; font-size: 13px; font-weight: bold;
                   min-width: 140px; min-height: 44px; border-radius: 5px; }
QPushButton#home:hover    { background: #006644; }
QPushButton#home:pressed  { background: #004422; }
QPushButton#home:disabled { background: #cccccc; border-color: #cccccc; color: #ffffff; }
QLineEdit {
    background: #ffffff;
    color: #111111;
    border: 1px solid #cccccc;
    border-radius: 4px;
    padding: 5px 8px;
    font-family: Arial;
    font-weight: bold;
    font-size: 14px;
}
QLineEdit:focus { border: 2px solid #005533; }
QComboBox {
    background: #ffffff; color: #111111;
    border: 1px solid #cccccc; border-radius: 4px;
    padding: 5px 8px; font-family: Arial; font-size: 12px;
}
QComboBox::drop-down { border: none; }
QTextEdit {
    background: #ffffff; color: #111111;
    border: 1px solid #dddddd; border-radius: 4px;
    font-family: Arial; font-size: 14px; padding: 6px;
}
"""


# ── Main Window ──────────────────────────────────────────────────
class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Linear Stage")
        self.setMinimumSize(520, 640)
        self.setStyleSheet(STYLE)
        self.tmcl     = None
        self.running  = False
        self.jogging  = False
        self.home_set = False   # must be True before START is allowed
        self.sig      = Sig()
        self.sig.log.connect(self._log)
        self.sig.status.connect(self._set_status)
        self.sig.done.connect(self._on_done)
        self.sig.cycles.connect(self._on_cycle)
        self._build()

    def _build(self):
        root = QWidget(); self.setCentralWidget(root)
        v = QVBoxLayout(root)
        v.setContentsMargins(16, 12, 16, 12)
        v.setSpacing(6)

        # ── Title row ─────────────────────────────────────────────
        h = QHBoxLayout()
        t = QLabel("LINEAR STAGE CONTROLLER")
        t.setFont(QFont("Arial", 14, QFont.Bold))
        t.setStyleSheet("color: #005533;")
        self.st_dot = QLabel("●")
        self.st_dot.setFont(QFont("Arial", 12))
        self.st_dot.setStyleSheet("color: #cc0000;")
        self.st_lbl = QLabel("DISCONNECTED")
        self.st_lbl.setFont(QFont("Arial", 10, QFont.Bold))
        self.st_lbl.setStyleSheet("color: #cc0000;")
        h.addWidget(t); h.addStretch()
        h.addWidget(self.st_lbl); h.addWidget(self.st_dot)
        v.addLayout(h)

        div = QFrame(); div.setFrameShape(QFrame.HLine)
        div.setStyleSheet("color: #dddddd;"); v.addWidget(div)

        # ── Connection ────────────────────────────────────────────
        cbox = QGroupBox("CONNECTION")
        cl = QHBoxLayout(cbox); cl.setSpacing(6)
        self.port_cb = QComboBox(); self.port_cb.setMinimumWidth(130)
        self._refresh_ports()
        r = QPushButton("↺"); r.setObjectName("sm"); r.setFixedWidth(36)
        r.clicked.connect(self._refresh_ports)
        con = QPushButton("CONNECT");    con.setObjectName("sm")
        dis = QPushButton("DISCONNECT"); dis.setObjectName("dis")
        con.clicked.connect(self._connect)
        dis.clicked.connect(self._disconnect)
        cl.addWidget(self.port_cb); cl.addWidget(r)
        cl.addStretch(); cl.addWidget(con); cl.addWidget(dis)
        v.addWidget(cbox)

        # ── Manual control + Set Home ─────────────────────────────
        jbox = QGroupBox("Hold to set HOME, click SET HOME when ready")
        jbox.setStyleSheet("QGroupBox { color: #000000; } QGroupBox::title { color: #000000; }")
        jl = QHBoxLayout(jbox); jl.setSpacing(12)

        self.jog_r_btn    = QPushButton("▲  HOME")
        self.jog_l_btn    = QPushButton("▼  END")
        self.set_home_btn = QPushButton("SET HOME")

        self.jog_r_btn.setObjectName("jog")
        self.jog_l_btn.setObjectName("jog")
        self.set_home_btn.setObjectName("home")

        self.jog_l_btn.pressed.connect(self._jog_left_start)
        self.jog_l_btn.released.connect(self._jog_stop)
        self.jog_r_btn.pressed.connect(self._jog_right_start)
        self.jog_r_btn.released.connect(self._jog_stop)
        self.set_home_btn.clicked.connect(self._set_home)

        jl.addWidget(self.jog_r_btn)
        jl.addWidget(self.jog_l_btn)
        jl.addWidget(self.set_home_btn)
        jl.addStretch()
        v.addWidget(jbox)
        self._set_jog(False)

        # ── Home status indicator ─────────────────────────────────
        self.home_lbl = QLabel("⚠  No home set — jog to position and click SET HOME")
        self.home_lbl.setFont(QFont("Arial", 11, QFont.Bold))
        self.home_lbl.setStyleSheet("color: #bb0000; padding: 2px 0;")
        v.addWidget(self.home_lbl)

        # ── Parameters ────────────────────────────────────────────
        pbox = QGroupBox("Set parameters and click START")
        pg = QGridLayout(pbox)
        pg.setVerticalSpacing(7)
        pg.setHorizontalSpacing(12)
        pg.setColumnMinimumWidth(0, 160)
        pg.setColumnMinimumWidth(1, 120)
        pg.setColumnMinimumWidth(2, 130)

        self.dist_e   = self._inp("10.0", QDoubleValidator(0.1, 300.0, 1))
        self.speed_e  = self._inp("120",  QDoubleValidator(0.1, 999.0, 1))
        self.rspeed_e = self._inp("120",  QDoubleValidator(0.1, 999.0, 1))
        self.hold_e   = self._inp("3.0",  QDoubleValidator(0.0, 999.0, 1))
        self.loops_e  = self._inp("0",    QIntValidator(1, 9999))

        params = [
            ("DISTANCE",      self.dist_e,   "cm from home"),
            ("FORWARD SPEED", self.speed_e,  "cm/min  (home → target)"),
            ("RETURN SPEED",  self.rspeed_e, "cm/min  (target → home)"),
            ("HOLD TIME",     self.hold_e,   "seconds at target"),
            ("LOOPS",         self.loops_e,  "1 – 9999"),
        ]

        for i, (name, widget, hint) in enumerate(params):
            nl = QLabel(name)
            nl.setFont(QFont("Arial", 14, QFont.Bold))
            nl.setStyleSheet("color: #111111;")
            hl = QLabel(hint)
            hl.setFont(QFont("Arial", 14))
            hl.setStyleSheet("color: #aaaaaa;")
            pg.addWidget(nl,     i, 0, Qt.AlignVCenter)
            pg.addWidget(widget, i, 1)
            pg.addWidget(hl,     i, 2, Qt.AlignVCenter)

        v.addWidget(pbox)

        # ── Cycle counter ─────────────────────────────────────────
        self.cycle_lbl = QLabel("Cycles completed:  —")
        self.cycle_lbl.setFont(QFont("Arial", 11, QFont.Bold))
        self.cycle_lbl.setStyleSheet("color: #005533; padding: 2px 0;")
        v.addWidget(self.cycle_lbl)

        # ── Start / Stop ──────────────────────────────────────────
        brow = QHBoxLayout(); brow.setSpacing(10)
        self.start_btn = QPushButton("▶   START")
        self.stop_btn  = QPushButton("■   STOP")
        self.stop_btn.setObjectName("stop")
        for b in (self.start_btn, self.stop_btn):
            b.setMinimumHeight(44)
            b.setFont(QFont("Arial", 13, QFont.Bold))
        self.start_btn.clicked.connect(self._start)
        self.stop_btn.clicked.connect(self._stop)
        self.stop_btn.setEnabled(False)
        brow.addWidget(self.start_btn); brow.addWidget(self.stop_btn)
        v.addLayout(brow)

        # ── Log ───────────────────────────────────────────────────
        lbox = QGroupBox("LOG")
        ll = QVBoxLayout(lbox); ll.setContentsMargins(6, 6, 6, 6)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(100)
        self.log_box.setMaximumHeight(140)
        ll.addWidget(self.log_box)
        v.addWidget(lbox)

        self._clear_fields()

    def _inp(self, default, validator=None):
        e = QLineEdit(str(default))
        e.setAlignment(Qt.AlignRight)
        e.setFixedWidth(120)
        e.setFont(QFont("Arial", 14, QFont.Bold))
        if validator: e.setValidator(validator)
        return e

    def _set_jog(self, en):
        self.jog_l_btn.setEnabled(en)
        self.jog_r_btn.setEnabled(en)
        self.set_home_btn.setEnabled(en)

    # ── Logging ───────────────────────────────────────────────────
    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_box.append(
            f'<span style="color:#999999">[{ts}]</span> '
            f'<span style="color:#000000">{msg}</span>')
        self.log_box.moveCursor(QTextCursor.End)

    def _set_status(self, msg, color):
        self.st_lbl.setText(msg)
        self.st_lbl.setStyleSheet(f"color:{color};")
        self.st_dot.setStyleSheet(f"color:{color};")

    def _val(self, field, fallback):
        try: return type(fallback)(field.text())
        except: return fallback

    def _on_cycle(self, n):
        self.cycle_lbl.setText(f"Cycles completed:  {n}")

    def _on_done(self):
        self.running  = False
        self.home_set = False   # require fresh SET HOME for next operation
        if self.tmcl:
            try: self.tmcl.mst()
            except: pass
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self._set_jog(True)
        self.home_lbl.setText("⚠  Cycle complete — set new home position to run again")
        self.home_lbl.setStyleSheet("color: #bb6600; padding: 2px 0;")
        self._log("All cycles complete — motor stopped ✓  Set home to run again.")

    # ── Connection ────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb.clear()
        self.port_cb.addItems(ports if ports else ["No ports found"])

    def _connect(self):
        port = self.port_cb.currentText()
        try:
            self.tmcl = TMCL(port)
            self.tmcl.sap(12, 1)
            self.tmcl.sap(13, 1)
            self.tmcl.sap(154, PULSE_DIV)
            self.sig.status.emit("CONNECTED", "#006644")
            self._log(f"Connected → {port}  |  Jog to position and click SET HOME")
            self._set_jog(True)
        except Exception as e:
            self._log(f"Failed: {e}")

    def _disconnect(self):
        self._stop()
        if self.tmcl:
            try: self.tmcl.mst()
            except: pass
            self.tmcl.close(); self.tmcl = None
        self.sig.status.emit("DISCONNECTED", "#cc0000")
        self._set_jog(False)
        self.home_set = False
        self.home_lbl.setText("⚠  No home set — jog to position and click SET HOME")
        self.home_lbl.setStyleSheet("color: #bb0000; padding: 2px 0;")
        self._log("Disconnected")
        self._clear_fields()

    # ── Set Home ──────────────────────────────────────────────────
    def _set_home(self):
        if not self.tmcl: return
        self.tmcl.zero_pos()
        self.home_set = True
        self.home_lbl.setText("✓  Home set at current position — ready to START")
        self.home_lbl.setStyleSheet("color: #005533; padding: 2px 0;")
        self._log("✓ Home position set — position counter zeroed")

    # ── Jog ───────────────────────────────────────────────────────
    def _jog_units(self):
        return 500   # fixed jog speed, independent of parameters

    def _jog_left_start(self):
        if not self.tmcl or self.running: return
        self.jogging = True
        self.tmcl.sap(6, CURRENT)
        self.tmcl.sap(5, ACCEL)
        self.tmcl.ror(self._jog_units())
        threading.Thread(target=self._jog_watch, args=('l',), daemon=True).start()

    def _jog_right_start(self):
        if not self.tmcl or self.running: return
        self.jogging = True
        self.tmcl.sap(6, CURRENT)
        self.tmcl.sap(5, ACCEL)
        self.tmcl.rol(self._jog_units())
        threading.Thread(target=self._jog_watch, args=('r',), daemon=True).start()

    def _jog_stop(self):
        self.jogging = False
        if self.tmcl:
            try: self.tmcl.mst()
            except: pass

    def _jog_watch(self, direction):
        t = self.tmcl
        while self.jogging and t:
            # Safety stops only — limit switches are fallback
            hit = (t.stop_r() == 0) if direction == 'l' else (t.stop_l() == 0)
            if hit:
                t.mst()
                self.jogging = False
                label = "STOP_R (end)" if direction == 'l' else "STOP_L (home end)"
                self.sig.log.emit(f"⚠ Safety limit hit — {label}")
                break
            time.sleep(0.03)

    # ── Motion ────────────────────────────────────────────────────
    def _apply(self):
        self.tmcl.sap(6,   CURRENT)
        self.tmcl.sap(5,   ACCEL)
        self.tmcl.sap(154, PULSE_DIV)

    def _start(self):
        if not self.tmcl:
            self._log("Not connected"); return
        if self.running: return
        if not self.home_set:
            self._log("⚠  Home not set — jog to desired position and click SET HOME first")
            return
        self._apply()
        self.running = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self._set_jog(False)
        self.cycle_lbl.setText("Cycles completed:  0")
        loops = self._val(self.loops_e, 0)
        dist  = self._val(self.dist_e,  10.0)
        spd   = self._val(self.speed_e, 120.0)
        rspd  = self._val(self.rspeed_e, 120.0)
        hold  = self._val(self.hold_e,  3.0)
        t_sec = travel_time(dist, spd)
        self._log(f"Starting  |  dist={dist}cm  fwd={spd}cm/min  rtn={rspd}cm/min  "
                  f"travel={t_sec:.1f}s  hold={hold}s  loops={loops}")
        threading.Thread(target=self._run, args=(loops,), daemon=True).start()

    def _stop(self):
        self.running  = False
        self.jogging  = False
        self.home_set = False   # require fresh SET HOME after any stop
        if self.tmcl:
            try: self.tmcl.mst()
            except: pass
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self._set_jog(True if self.tmcl else False)
        self.home_lbl.setText("⚠  Stopped — set home again before next run")
        self.home_lbl.setStyleSheet("color: #bb0000; padding: 2px 0;")
        self._log("Stopped — set home position again before next run")

    def _clear_fields(self):
      self.dist_e.setText("")
      self.speed_e.setText("")
      self.rspeed_e.setText("")
      self.hold_e.setText("")
      self.loops_e.setText("")

    # ── Run loop ──────────────────────────────────────────────────
    def _run(self, loops):
        t   = self.tmcl
        inf = (loops == 0)

        # Stage is already at home (user set it) — no homing move needed
        self.sig.log.emit("At home position — beginning cycle")

        count = 0
        while self.running and t:
            try:
                dist  = self._val(self.dist_e,   10.0)
                spd   = self._val(self.speed_e,  120.0)
                rspd  = self._val(self.rspeed_e, 120.0)
                hold  = self._val(self.hold_e,    3.0)
                t_sec = travel_time(dist, spd)

                # ── Move to target (forward) ───────────────────────
                self.sig.log.emit(f"Moving to target  {dist}cm at {spd}cm/min ...")
                t.ror(cmm_to_units(spd))
                t_start = time.time()
                while self.running and (time.time() - t_start) < t_sec:
                    if t.stop_r() == 0:           # safety only
                        t.mst()
                        self.sig.log.emit("⚠ STOP_R safety hit — stopping")
                        self.sig.done.emit()
                        return
                    time.sleep(0.02)
                t.mst()
                if not self.running: break

                # ── Hold at target ────────────────────────────────
                self.sig.log.emit(f"At target {dist}cm — holding {hold}s")
                t_hold = time.time()
                while self.running and (time.time() - t_hold) < hold:
                    time.sleep(0.05)
                if not self.running: break

                # ── Return to home (position-based) ───────────────
                self.sig.log.emit(f"Returning to home at {rspd}cm/min ...")
                t.rol(cmm_to_units(rspd))
                while self.running:
                    if t.stop_l() == 0:           # safety only
                        t.mst()
                        self.sig.log.emit("⚠ STOP_L safety hit on return — stopping")
                        self.sig.done.emit()
                        return
                    pos = t.get_pos()
                    if pos is not None and pos <= HOME_THRESHOLD:
                        t.mst()
                        t.zero_pos()              # re-zero precisely at home
                        break
                    time.sleep(0.02)
                if not self.running: break

                # ── Cycle complete ────────────────────────────────
                count += 1
                self.sig.cycles.emit(count)
                self.sig.log.emit(f"Cycle {count} complete — back at home")

                if count >= loops:
                    self.sig.done.emit()
                    return

            except Exception as e:
                self.sig.log.emit(f"Error: {e}"); break

    def closeEvent(self, e):
        self._disconnect(); e.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = App(); win.show()
    sys.exit(app.exec_())