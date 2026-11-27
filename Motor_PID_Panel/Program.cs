using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Windows.Forms;

namespace MotorPidPanel
{
    internal static class Protocol
    {
        public static string Number(string text, decimal min, decimal max, bool integer)
        {
            decimal value;
            text = (text ?? "").Trim().Replace(',', '.');
            if (!decimal.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value)
                || value < min || value > max || (integer && decimal.Truncate(value) != value))
                throw new FormatException("Giá trị phải trong khoảng " + min + " đến " + max + (integer ? " và là số nguyên." : "."));
            return value.ToString("0.######", CultureInfo.InvariantCulture);
        }
        public static string Pid(string p, string i, string d)
        {
            return "PID " + Number(p, 0, 200, false) + " " + Number(i, 0, 50, false) + " " + Number(d, 0, 20, false);
        }
        public static string PosPid(string p, string i, string d)
        {
            return "PPID " + Number(p, 0, 50, false) + " " + Number(i, 0, 10, false) + " " + Number(d, 0, 50, false);
        }
        public static string YawPid(string p, string d)
        {
            return "YPID " + Number(p, 0, 5, false) + " " + Number(d, 0, 10, false);
        }
        public static string Motor(decimal a, decimal b)
        {
            return "M " + Number(a.ToString(CultureInfo.InvariantCulture), -999, 999, true) + " "
                + Number(b.ToString(CultureInfo.InvariantCulture), -999, 999, true);
        }
        public static Dictionary<string, string> Fields(string line)
        {
            var result = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (string part in line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries))
            {
                int split = part.IndexOf('=');
                if (split > 0) result[part.Substring(0, split)] = part.Substring(split + 1);
            }
            return result;
        }
        public static bool NeedsStopped(string cmd)
        {
            string word = cmd.Split(' ')[0].ToUpperInvariant();
            return word == "PID" || word == "PPID" || word == "SPID" || word == "YPID"
                || word == "ZERO" || word == "SETZERO" || word == "LIMIT" || word == "DEAD"
                || word == "RAMP" || word == "ENC" || word == "CPR"
                || word == "DIR" || word == "SIGN" || word == "EDIR" || word == "YSIGN" || word == "AXIS";
        }
        public static bool RequiresMotorOff(string cmd)
        {
            string word = cmd.Split(' ')[0].ToUpperInvariant();
            return word == "DIR" || word == "SIGN" || word == "EDIR" || word == "YSIGN" || word == "AXIS";
        }
        public static bool IsPidGreeting(string line)
        {
            return line.StartsWith("PID_PANEL PROTO=3 ", StringComparison.Ordinal);
        }
        private static bool ValidNumber(Dictionary<string, string> fields, string key, decimal min, decimal max, bool integer)
        {
            string text; decimal value;
            return fields.TryGetValue(key, out text)
                && decimal.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value)
                && value >= min && value <= max && (!integer || decimal.Truncate(value) == value);
        }
        public static bool IsStatus(string line, Dictionary<string, string> fields)
        {
            string mode;
            return line.StartsWith("MODE=", StringComparison.Ordinal)
                && fields.TryGetValue("MODE", out mode)
                && (mode == "STOP" || mode == "MANUAL" || mode == "BALANCE" || mode == "FAULT")
                && fields.ContainsKey("IMU") && fields.ContainsKey("REASON")
                && ValidNumber(fields, "ANG", -180, 180, false)
                && ValidNumber(fields, "RATE", -1000, 1000, false)
                && ValidNumber(fields, "A", -999, 999, true)
                && ValidNumber(fields, "B", -999, 999, true);
        }
    }

    internal sealed class LineReader
    {
        private readonly StringBuilder pending = new StringBuilder();
        private bool dropping;
        public void Reset() { pending.Length = 0; dropping = false; }
        public void Push(string data, Action<string> onLine)
        {
            foreach (char c in data)
            {
                if (c == '\r' || c == '\n')
                {
                    string complete = !dropping && pending.Length > 0 ? pending.ToString() : null;
                    pending.Length = 0; dropping = false;
                    if (complete != null) onLine(complete);
                }
                else if (!dropping)
                {
                    if (pending.Length < 2048) pending.Append(c);
                    else { dropping = true; pending.Length = 0; }
                }
            }
        }
    }

    internal sealed class TelemetryChart : Control
    {
        private const int MaxSamples = 120;
        private readonly List<float> tiltSeries = new List<float>(MaxSamples);
        private readonly List<float> trefSeries = new List<float>(MaxSamples);
        private readonly List<float> pwmSeries = new List<float>(MaxSamples);

        public TelemetryChart()
        {
            SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
            BackColor = Color.White;
        }
        public void Clear()
        {
            tiltSeries.Clear(); trefSeries.Clear(); pwmSeries.Clear();
            Invalidate();
        }
        public void AddSample(float tiltDeg, float trefDeg, float pwmScaledDeg)
        {
            if (tiltSeries.Count >= MaxSamples)
            {
                tiltSeries.RemoveAt(0); trefSeries.RemoveAt(0); pwmSeries.RemoveAt(0);
            }
            tiltSeries.Add(Math.Max(-25f, Math.Min(25f, tiltDeg)));
            trefSeries.Add(Math.Max(-25f, Math.Min(25f, trefDeg)));
            pwmSeries.Add(Math.Max(-25f, Math.Min(25f, pwmScaledDeg)));
            Invalidate();
        }
        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            int w = ClientSize.Width, h = ClientSize.Height;
            if (w < 40 || h < 40) return;

            var rect = new Rectangle(34, 22, w - 42, h - 30);
            using (var borderPen = new Pen(Color.FromArgb(220, 226, 232)))
                g.DrawRectangle(borderPen, rect);

            using (var gridPen = new Pen(Color.FromArgb(236, 240, 244)))
            using (var zeroPen = new Pen(Color.FromArgb(190, 198, 208)))
            using (var axisBrush = new SolidBrush(Color.FromArgb(100, 110, 120)))
            using (var smallFont = new Font("Segoe UI", 7.5f))
            {
                foreach (int deg in new int[] { -20, -10, 0, 10, 20 })
                {
                    float y = rect.Top + rect.Height * (25f - deg) / 50f;
                    g.DrawLine(deg == 0 ? zeroPen : gridPen, rect.Left, y, rect.Right, y);
                    g.DrawString(deg.ToString(CultureInfo.InvariantCulture) + "°", smallFont, axisBrush, 2, y - 6f);
                }
                using (var b1 = new SolidBrush(Color.DodgerBlue))
                using (var b2 = new SolidBrush(Color.DarkOrange))
                using (var b3 = new SolidBrush(Color.SeaGreen))
                {
                    g.DrawString("● Góc lệch (°)", smallFont, b1, rect.Left + 2, 4);
                    g.DrawString("● θ_ref (°)", smallFont, b2, rect.Left + 88, 4);
                    g.DrawString("● PWM/50", smallFont, b3, rect.Left + 158, 4);
                }
            }

            DrawSeries(g, rect, pwmSeries, Color.SeaGreen, 1.4f);
            DrawSeries(g, rect, trefSeries, Color.DarkOrange, 1.6f);
            DrawSeries(g, rect, tiltSeries, Color.DodgerBlue, 2.0f);
        }
        private static void DrawSeries(Graphics g, Rectangle rect, List<float> data, Color color, float width)
        {
            if (data.Count < 2) return;
            var pts = new PointF[data.Count];
            for (int i = 0; i < data.Count; i++)
            {
                float x = rect.Left + (float)i * rect.Width / (MaxSamples - 1);
                float y = rect.Top + rect.Height * (25f - data[i]) / 50f;
                pts[i] = new PointF(x, y);
            }
            using (var pen = new Pen(color, width))
                g.DrawLines(pen, pts);
        }
    }

    internal sealed class PanelForm : Form
    {
        private SerialPort port;
        private readonly LineReader reader = new LineReader();
        private readonly ComboBox ports = new ComboBox();
        private readonly Button connect = new Button();
        private readonly Button stop = new Button();
        private readonly Label angle = new Label(), state = new Label(), imu = new Label();
        private readonly Label rate = new Label(), pwm = new Label(), reason = new Label(), age = new Label();
        private readonly Label connection = new Label();
        private readonly Label actualPid = new Label(), actualOuter = new Label();
        private readonly TelemetryChart chart = new TelemetryChart();
        private readonly TextBox kp = new TextBox(), ki = new TextBox(), kd = new TextBox();
        private readonly TextBox pkp = new TextBox(), pki = new TextBox(), pkd = new TextBox();
        private readonly TextBox ykp = new TextBox(), ykd = new TextBox();
        private readonly TextBox zero = new TextBox(), limit = new TextBox(), dead = new TextBox(), ramp = new TextBox();
        private readonly ComboBox dirA = new ComboBox(), dirB = new ComboBox(), sign = new ComboBox();
        private readonly ComboBox edirL = new ComboBox(), edirR = new ComboBox(), ysign = new ComboBox();
        private readonly NumericUpDown motorA = new NumericUpDown(), motorB = new NumericUpDown();
        private readonly TextBox raw = new TextBox();
        private readonly TextBox log = new TextBox();
        private readonly CheckBox autoStatus = new CheckBox(), logTelemetry = new CheckBox();
        private readonly List<Control> commandControls = new List<Control>();
        private readonly Timer timer = new Timer();
        private DateTime lastStatus = DateTime.MinValue, nextPoll = DateTime.MinValue;
        private DateTime calibratingUntil = DateTime.MinValue, closeAt = DateTime.MaxValue;
        private DateTime handshakeDeadline;
        private bool awaitingStop, closingPort, closeWindow, applyNextSettings, preview;
        private bool protocolReady, statusRequested;
        private string reportedMode = "UNKNOWN";
        private string firmwareVersion = "";
        private string lastWho = "0x12";
        public bool Connected { get { return port != null && port.IsOpen; } }

        public PanelForm()
        {
            Text = "Motor PID Panel v4.2 (6.0V CAD) · STM32 + ICM-20602 + TB6612";
            Font = new Font("Segoe UI", 9.0f);
            AutoScaleMode = AutoScaleMode.Dpi;
            ClientSize = new Size(1220, 910);
            MinimumSize = new Size(1040, 700);
            StartPosition = FormStartPosition.CenterScreen;
            BackColor = Color.FromArgb(245, 247, 250);
            KeyPreview = true;
            KeyDown += delegate(object sender, KeyEventArgs e) {
                if (e.KeyCode == Keys.Escape) { EmergencyStop(); e.SuppressKeyPress = true; }
            };
            BuildLayout();
            RefreshPorts();
            timer.Interval = 50;
            timer.Tick += Poll;
            timer.Start();
            FormClosing += OnClosing;
            SetConnectionState();
            AddLog("v4.2 (VM=6.0V): Preset chuẩn từ Khung_Frame.step (92/0/5.5 | PPID 9.0/1.8/12.5 | YPID 0.30/1.20 | DB110/R48).", false);
        }
        private static Label Caption(string text)
        {
            return new Label { Text = text, AutoSize = true, Margin = new Padding(4, 7, 2, 2) };
        }
        private Button Button(string text, EventHandler action, bool command)
        {
            var b = new Button { Text = text, AutoSize = true, MinimumSize = new Size(64, 28), Margin = new Padding(3), Padding = new Padding(4, 1, 4, 1) };
            b.Click += action;
            if (command) commandControls.Add(b);
            return b;
        }
        private void AddNudge(FlowLayoutPanel panel, TextBox box, decimal delta, decimal min, decimal max, bool integer, string fmt, Func<string> buildCmd)
        {
            var minus = new Button { Text = "-", Size = new Size(22, 24), Margin = new Padding(1, 5, 1, 4), Padding = new Padding(0), FlatStyle = FlatStyle.Standard };
            var plus = new Button { Text = "+", Size = new Size(22, 24), Margin = new Padding(1, 5, 4, 4), Padding = new Padding(0), FlatStyle = FlatStyle.Standard };
            minus.Click += delegate { NudgeValue(box, -delta, min, max, integer, fmt, buildCmd); };
            plus.Click += delegate { NudgeValue(box, delta, min, max, integer, fmt, buildCmd); };
            commandControls.Add(minus);
            commandControls.Add(plus);
            panel.Controls.Add(minus);
            panel.Controls.Add(plus);
        }
        private void NudgeValue(TextBox box, decimal delta, decimal min, decimal max, bool integer, string fmt, Func<string> buildCmd)
        {
            Attempt(delegate {
                decimal val;
                string txt = (box.Text ?? "0").Trim().Replace(',', '.');
                if (!decimal.TryParse(txt, NumberStyles.Float, CultureInfo.InvariantCulture, out val))
                    throw new FormatException("Không đọc được giá trị số hiện tại.");
                val = Math.Max(min, Math.Min(max, val + delta));
                if (integer) val = decimal.Round(val, 0);
                box.Text = val.ToString(fmt, CultureInfo.InvariantCulture);
                SendCommand(buildCmd());
            });
        }
        private void BindEnter(TextBox box, Func<string> buildCmd)
        {
            box.KeyDown += delegate(object sender, KeyEventArgs e) {
                if (e.KeyCode == Keys.Enter)
                {
                    e.SuppressKeyPress = true;
                    Attempt(delegate { SendCommand(buildCmd()); });
                }
            };
        }
        private void ApplyPreset(string name, string vkp, string vki, string vkd, string vpkp, string vpki, string vpkd, string vdead, string vramp)
        {
            kp.Text = vkp; ki.Text = vki; kd.Text = vkd;
            pkp.Text = vpkp; pki.Text = vpki; pkd.Text = vpkd;
            dead.Text = vdead; ramp.Text = vramp;
            if (Connected && protocolReady)
            {
                Attempt(delegate {
                    if (SendCommand(Protocol.Pid(kp.Text, ki.Text, kd.Text))
                        && SendCommand(Protocol.PosPid(pkp.Text, pki.Text, pkd.Text))
                        && SendCommand("DEAD " + Protocol.Number(dead.Text, 0, 250, true)))
                    {
                        SendCommand("RAMP " + Protocol.Number(ramp.Text, 1, 100, true));
                    }
                });
            }
            AddLog("Đã nạp Preset " + name + " (PID=" + vkp + "," + vki + "," + vkd + " | PPID=" + vpkp + "," + vpki + "," + vpkd + " | DEAD=" + vdead + ", RAMP=" + vramp + ").", false);
        }
        private static FlowLayoutPanel Flow()
        {
            return new FlowLayoutPanel { Dock = DockStyle.Fill, WrapContents = true, AutoSize = false, Padding = new Padding(4) };
        }
        private static GroupBox Group(string title, Control content)
        {
            var box = new GroupBox { Text = title, Dock = DockStyle.Fill, Padding = new Padding(6), Margin = new Padding(3) };
            box.Controls.Add(content); return box;
        }
        private void BuildLayout()
        {
            var host = new Panel { Dock = DockStyle.Fill, AutoScroll = true };
            var root = new TableLayoutPanel { Dock = DockStyle.Top, Height = 900, MinimumSize = new Size(1020, 900), ColumnCount = 1, RowCount = 9, Padding = new Padding(8) };
            foreach (int height in new int[] { 44, 152, 166, 114, 86, 44, 38 }) root.RowStyles.Add(new RowStyle(SizeType.Absolute, height));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 25));
            host.Controls.Add(root); Controls.Add(host);
            host.SizeChanged += delegate { root.Height = Math.Max(900, host.ClientSize.Height); };

            var top = Flow();
            top.Controls.Add(Caption("Cổng COM"));
            ports.Width = 90; ports.DropDownStyle = ComboBoxStyle.DropDownList; ports.Margin = new Padding(4, 5, 4, 4);
            top.Controls.Add(ports);
            top.Controls.Add(Button("Tìm cổng", delegate { RefreshPorts(); }, false));
            connect.Text = "Kết nối"; connect.AutoSize = true; connect.Height = 29; connect.Margin = new Padding(4, 3, 4, 3);
            connect.Click += delegate { if (Connected) BeginDisconnect(false); else OpenPort(); };
            top.Controls.Add(connect);
            autoStatus.Text = "Tự đọc (150ms)"; autoStatus.Checked = true; autoStatus.AutoSize = true; autoStatus.Margin = new Padding(8, 8, 4, 4);
            top.Controls.Add(autoStatus);
            top.Controls.Add(Button("Đọc STM32", delegate { if (SendCommand("STATUS")) applyNextSettings = true; }, true));
            top.Controls.Add(Button("Copy board_config.h", delegate { CopyBoardConfig(); }, false));
            top.Controls.Add(Button("Preset Êm", delegate { ApplyPreset("Êm 6V", "82.00", "0.00", "5.00", "7.50", "1.40", "11.00", "100", "50"); }, false));
            top.Controls.Add(Button("Preset Chuẩn", delegate { ApplyPreset("Chuẩn 6V", "92.00", "0.00", "5.50", "9.00", "1.80", "12.50", "110", "48"); }, false));
            top.Controls.Add(Button("Preset Cứng", delegate { ApplyPreset("Cứng 6V", "102.00", "0.00", "6.00", "10.00", "2.00", "13.50", "118", "45"); }, false));
            root.Controls.Add(top, 0, 0);

            var displayRow = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, BackColor = Color.White, Padding = new Padding(6), Margin = new Padding(3) };
            displayRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 46));
            displayRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 54));

            var display = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 4, Padding = new Padding(4) };
            display.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 46)); display.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 54));
            angle.Text = "Góc: —"; angle.Font = new Font("Segoe UI", 22, FontStyle.Bold); angle.Dock = DockStyle.Fill;
            display.Controls.Add(angle, 0, 0); display.SetRowSpan(angle, 2);
            state.Text = "CHƯA KẾT NỐI"; state.Font = new Font("Segoe UI", 11, FontStyle.Bold); state.AutoSize = true;
            imu.Text = "IMU: —"; imu.AutoSize = true;
            rate.Text = "Gyro P/Z: —"; rate.AutoSize = true;
            pwm.Text = "PWM A/B: —"; pwm.AutoSize = true;
            reason.Text = "Vị trí & Lý do: —"; reason.AutoSize = true;
            age.Text = "Dữ liệu: chưa nhận"; age.AutoSize = true;
            display.Controls.Add(state, 1, 0);
            display.Controls.Add(imu, 1, 1);
            display.Controls.Add(rate, 0, 2); display.Controls.Add(pwm, 1, 2);
            display.Controls.Add(reason, 0, 3); display.Controls.Add(age, 1, 3);
            displayRow.Controls.Add(display, 0, 0);

            chart.Dock = DockStyle.Fill;
            displayRow.Controls.Add(chart, 1, 0);
            root.Controls.Add(displayRow, 0, 1);

            var pidRow = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3 };
            pidRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 32));
            pidRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 44));
            pidRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 24));

            var pid = Flow();
            SetEntry(kp, "92.00", 50); SetEntry(ki, "0.00", 42); SetEntry(kd, "5.50", 46);
            Func<string> buildPid = delegate { return Protocol.Pid(kp.Text, ki.Text, kd.Text); };
            BindEnter(kp, buildPid); BindEnter(ki, buildPid); BindEnter(kd, buildPid);
            pid.Controls.Add(Caption("Kp")); pid.Controls.Add(kp); AddNudge(pid, kp, 4m, 0m, 200m, false, "0.00", buildPid);
            pid.Controls.Add(Caption("Ki")); pid.Controls.Add(ki);
            pid.Controls.Add(Caption("Kd")); pid.Controls.Add(kd); AddNudge(pid, kd, 0.4m, 0m, 20m, false, "0.00", buildPid);
            pid.SetFlowBreak(pid.Controls[pid.Controls.Count - 1], true);
            pid.Controls.Add(Button("Gửi PID", delegate { Attempt(delegate { SendCommand(buildPid()); }); }, true));
            pid.Controls.Add(Caption("(Enter gửi · Ki=0: PD thuần)"));
            pid.SetFlowBreak(pid.Controls[pid.Controls.Count - 1], true);
            actualPid.Text = "STM32 PID: chưa đọc"; actualPid.AutoSize = true;
            actualPid.Margin = new Padding(4, 4, 4, 0); pid.Controls.Add(actualPid);
            pidRow.Controls.Add(Group("Vòng trong: PD/PID Góc (200 Hz)", pid), 0, 0);

            var outer = Flow();
            SetEntry(pkp, "9.00", 44); SetEntry(pki, "1.80", 42); SetEntry(pkd, "12.50", 46);
            Func<string> buildPpid = delegate { return Protocol.PosPid(pkp.Text, pki.Text, pkd.Text); };
            BindEnter(pkp, buildPpid); BindEnter(pki, buildPpid); BindEnter(pkd, buildPpid);
            outer.Controls.Add(Caption("Kp_p")); outer.Controls.Add(pkp); AddNudge(outer, pkp, 1.0m, 0m, 50m, false, "0.00", buildPpid);
            outer.Controls.Add(Caption("Ki_p")); outer.Controls.Add(pki);
            outer.Controls.Add(Caption("Kd_p")); outer.Controls.Add(pkd); AddNudge(outer, pkd, 1.0m, 0m, 50m, false, "0.00", buildPpid);
            var ppidBtn = Button("Gửi PPID", delegate { Attempt(delegate { SendCommand(buildPpid()); }); }, true);
            outer.Controls.Add(ppidBtn);
            outer.SetFlowBreak(ppidBtn, true);

            SetEntry(ykp, "0.30", 44); SetEntry(ykd, "1.20", 44);
            Func<string> buildYpid = delegate { return Protocol.YawPid(ykp.Text, ykd.Text); };
            BindEnter(ykp, buildYpid); BindEnter(ykd, buildYpid);
            outer.Controls.Add(Caption("Kp_yaw")); outer.Controls.Add(ykp);
            outer.Controls.Add(Caption("Kd_yaw")); outer.Controls.Add(ykd);
            outer.Controls.Add(Button("Gửi YPID", delegate { Attempt(delegate { SendCommand(buildYpid()); }); }, true));
            var enc1Btn = Button("ENC 1", delegate { SendCommand("ENC 1"); }, true);
            enc1Btn.MinimumSize = new Size(52, 28); enc1Btn.Margin = new Padding(2, 3, 2, 3);
            var enc0Btn = Button("ENC 0", delegate { SendCommand("ENC 0"); }, true);
            enc0Btn.MinimumSize = new Size(52, 28); enc0Btn.Margin = new Padding(2, 3, 2, 3);
            outer.Controls.Add(enc1Btn);
            outer.Controls.Add(enc0Btn);
            outer.SetFlowBreak(enc0Btn, true);
            actualOuter.Text = "STM32 PPID/YPID: chưa đọc"; actualOuter.AutoSize = true;
            actualOuter.Margin = new Padding(4, 4, 4, 0); outer.Controls.Add(actualOuter);
            pidRow.Controls.Add(Group("Vòng ngoài: Vị trí CoM (50 Hz) && Yaw (200 Hz)", outer), 1, 0);

            var run = Flow();
            var arm = Button("BẬT CÂN BẰNG (ARM)", delegate { SendCommand("ARM"); }, true);
            arm.BackColor = Color.FromArgb(210, 239, 220); arm.Height = 32; run.Controls.Add(arm);
            run.Controls.Add(Button("Calib IMU (C)", delegate { SendCommand("C"); }, true));
            run.Controls.Add(Button("Góc hiện tại = ZERO", delegate { SendCommand("SETZERO"); }, true));
            stop.Text = "DỪNG NGAY — Esc"; stop.AutoSize = false; stop.Size = new Size(258, 34);
            stop.BackColor = Color.FromArgb(185, 36, 43); stop.ForeColor = Color.White; stop.FlatStyle = FlatStyle.Flat;
            stop.Font = new Font("Segoe UI", 10.5f, FontStyle.Bold); stop.Margin = new Padding(3);
            stop.Click += delegate { EmergencyStop(); }; run.Controls.Add(stop);
            pidRow.Controls.Add(Group("Điều khiển && An toàn", run), 2, 0);
            root.Controls.Add(pidRow, 0, 2);

            var settings = Flow();
            SetEntry(zero, "1.50", 48); SetEntry(dead, "110", 44); SetEntry(ramp, "48", 42); SetEntry(limit, "999", 46);
            Func<string> buildZero = delegate { return "ZERO " + Protocol.Number(zero.Text, -15, 15, false); };
            Func<string> buildDead = delegate { return "DEAD " + Protocol.Number(dead.Text, 0, 250, true); };
            Func<string> buildRamp = delegate { return "RAMP " + Protocol.Number(ramp.Text, 1, 100, true); };
            Func<string> buildLimit = delegate { return "LIMIT " + Protocol.Number(limit.Text, 1, 999, true); };
            BindEnter(zero, buildZero); BindEnter(dead, buildDead); BindEnter(ramp, buildRamp); BindEnter(limit, buildLimit);

            settings.Controls.Add(Caption("ZERO °")); settings.Controls.Add(zero); AddNudge(settings, zero, 0.20m, -15m, 15m, false, "0.00", buildZero);
            settings.Controls.Add(Button("Đặt ZERO", delegate { Attempt(delegate { SendCommand(buildZero()); }); }, true));
            settings.Controls.Add(Caption("DEAD")); settings.Controls.Add(dead); AddNudge(settings, dead, 5m, 0m, 250m, true, "0", buildDead);
            settings.Controls.Add(Button("Đặt DEAD", delegate { Attempt(delegate { SendCommand(buildDead()); }); }, true));
            settings.Controls.Add(Caption("RAMP")); settings.Controls.Add(ramp);
            settings.Controls.Add(Button("Đặt RAMP", delegate { Attempt(delegate { SendCommand(buildRamp()); }); }, true));
            settings.Controls.Add(Caption("LIMIT")); settings.Controls.Add(limit);
            var limitBtn = Button("Đặt LIMIT", delegate { Attempt(delegate { SendCommand(buildLimit()); }); }, true);
            settings.Controls.Add(limitBtn); settings.SetFlowBreak(limitBtn, true);

            SetSign(dirA, "-1"); SetSign(dirB, "-1"); SetSign(sign, "-1");
            SetSign(edirL, "1"); SetSign(edirR, "-1"); SetSign(ysign, "-1");
            settings.Controls.Add(Caption("DIR A/B")); settings.Controls.Add(dirA); settings.Controls.Add(dirB);
            settings.Controls.Add(Button("Đặt DIR", delegate { SendCommand("DIR " + dirA.Text + " " + dirB.Text); }, true));
            settings.Controls.Add(Caption("SIGN")); settings.Controls.Add(sign);
            settings.Controls.Add(Button("Đặt SIGN", delegate { SendCommand("SIGN " + sign.Text); }, true));
            settings.Controls.Add(Caption("EDIR L/R")); settings.Controls.Add(edirL); settings.Controls.Add(edirR);
            settings.Controls.Add(Button("Đặt EDIR", delegate { SendCommand("EDIR " + edirL.Text + " " + edirR.Text); }, true));
            settings.Controls.Add(Caption("YSIGN")); settings.Controls.Add(ysign);
            settings.Controls.Add(Button("Đặt YSIGN", delegate { SendCommand("YSIGN " + ysign.Text); }, true));
            root.Controls.Add(Group("Thiết lập cân bằng (Hàng 1: live + Enter + -/+ · Hàng 2: chỉ khi motor dừng)", settings), 0, 3);

            var motors = Flow();
            foreach (NumericUpDown input in new NumericUpDown[] { motorA, motorB })
            { input.Minimum = -999; input.Maximum = 999; input.Increment = 50; input.Value = 400; input.Width = 76; input.Margin = new Padding(4, 5, 4, 4); }
            motors.Controls.Add(Caption("PWM A")); motors.Controls.Add(motorA); motors.Controls.Add(Caption("PWM B")); motors.Controls.Add(motorB);
            motors.Controls.Add(Button("Thử A", delegate { SendCommand(Protocol.Motor(motorA.Value, 0)); }, true));
            motors.Controls.Add(Button("Thử B", delegate { SendCommand(Protocol.Motor(0, motorB.Value)); }, true));
            motors.Controls.Add(Button("Thử A + B", delegate { SendCommand(Protocol.Motor(motorA.Value, motorB.Value)); }, true));
            motors.Controls.Add(Caption("Mỗi lần thử 2s · ±999 · nâng bánh khi thử."));
            root.Controls.Add(Group("Thử motor riêng — M a b (không cần IMU)", motors), 0, 4);

            var direct = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, Padding = new Padding(4, 6, 4, 4) };
            direct.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 90)); direct.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100)); direct.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 95));
            direct.Controls.Add(Caption("Lệnh khác:"), 0, 0); raw.Dock = DockStyle.Fill;
            raw.KeyDown += delegate(object sender, KeyEventArgs e) { if (e.KeyCode == Keys.Enter) { SendRawBox(); e.SuppressKeyPress = true; } };
            direct.Controls.Add(raw, 1, 0); direct.Controls.Add(Button("Gửi lệnh", delegate { SendRawBox(); }, true), 2, 0);
            root.Controls.Add(direct, 0, 5);

            var logBar = Flow();
            logTelemetry.Text = "Ghi telemetry vào nhật ký"; logTelemetry.AutoSize = true; logTelemetry.Margin = new Padding(4, 5, 10, 0); logBar.Controls.Add(logTelemetry);
            logBar.Controls.Add(Button("Xóa nhật ký", delegate { log.Clear(); }, false));
            logBar.Controls.Add(Button("Xóa đồ thị", delegate { chart.Clear(); }, false));
            logBar.Controls.Add(Button("Lưu nhật ký…", delegate { SaveLog(); }, false));
            root.Controls.Add(logBar, 0, 6);

            log.Dock = DockStyle.Fill; log.Multiline = true; log.ScrollBars = ScrollBars.Vertical;
            log.ReadOnly = true; log.BackColor = Color.White; log.Font = new Font("Consolas", 9);
            root.Controls.Add(log, 0, 7);
            connection.Dock = DockStyle.Fill; connection.TextAlign = ContentAlignment.MiddleLeft; root.Controls.Add(connection, 0, 8);
        }
        private static void SetEntry(TextBox box, string text, int width)
        { box.Text = text; box.Width = width; box.Margin = new Padding(2, 5, 2, 4); }
        private static void SetSign(ComboBox box, string defaultVal)
        {
            box.DropDownStyle = ComboBoxStyle.DropDownList;
            box.Items.AddRange(new object[] { "1", "-1" });
            box.SelectedItem = defaultVal;
            box.Width = 52;
            box.Margin = new Padding(3, 5, 3, 4);
        }
        private void Attempt(Action action)
        { try { action(); } catch (FormatException ex) { AddLog(ex.Message, true); } }
        private void RefreshPorts()
        {
            if (Connected) return;
            string before = ports.Text;
            ports.Items.Clear(); string[] names = SerialPort.GetPortNames(); Array.Sort(names);
            ports.Items.AddRange(names);
            int selected = ports.Items.IndexOf(before);
            if (selected < 0) selected = ports.Items.IndexOf("COM5");
            if (selected < 0 && ports.Items.Count > 0) selected = 0;
            ports.SelectedIndex = selected;
        }
        private void OpenPort()
        {
            if (String.IsNullOrEmpty(ports.Text)) { AddLog("Chọn cổng COM của ST-LINK trước.", true); return; }
            try
            {
                port = new SerialPort(ports.Text, 115200, Parity.None, 8, StopBits.One);
                port.Handshake = Handshake.None; port.DtrEnable = false; port.RtsEnable = false;
                port.ReadTimeout = 100; port.WriteTimeout = 150; port.Encoding = Encoding.ASCII;
                port.Open(); reader.Reset(); reportedMode = "UNKNOWN"; lastStatus = DateTime.MinValue;
                awaitingStop = false; closingPort = false; firmwareVersion = ""; protocolReady = false;
                applyNextSettings = true; statusRequested = false; chart.Clear();
                angle.Text = "Góc: —"; imu.Text = "IMU: —";
                state.Text = "ĐANG KẾT NỐI"; age.Text = "Đang đợi trạng thái từ STM32";
                handshakeDeadline = DateTime.UtcNow.AddSeconds(10);
                nextPoll = DateTime.UtcNow.AddMilliseconds(150);
                SetConnectionState(); AddLog("Đã mở " + port.PortName + ". Đang nhận diện firmware PID bằng ?; không tự bật motor.", false);
                port.Write("?\r");
            }
            catch (Exception ex) { DropPort(); AddLog("Không mở được COM: " + ex.Message + " Đóng PuTTY/Serial Monitor nếu đang mở.", true); }
        }
        private void SetConnectionState()
        {
            bool usable = ((Connected && protocolReady) || preview) && !closingPort && !awaitingStop && DateTime.UtcNow >= calibratingUntil;
            foreach (Control c in commandControls) c.Enabled = usable;
            stop.Enabled = ((Connected && protocolReady) || preview) && !closingPort; ports.Enabled = !Connected;
            connect.Enabled = !closingPort; connect.Text = Connected ? "Ngắt kết nối" : "Kết nối";
            raw.Enabled = usable;
            connection.Text = Connected ? port.PortName + " · 115200 · " + (protocolReady ? firmwareVersion + " · ICM-20602 (" + lastWho + ")" : "đang đợi nhận diện firmware PID")
                : "Chưa kết nối. Đóng PuTTY → chọn COM của ST-LINK → Kết nối.";
        }
        private void SendRawBox()
        {
            string cmd = raw.Text.Trim();
            if (cmd.Length == 0) return;
            if (SendCommand(cmd)) { raw.Clear(); raw.Focus(); }
        }
        private bool SendCommand(string cmd)
        {
            cmd = cmd.Trim();
            if (cmd.Equals("X", StringComparison.OrdinalIgnoreCase)) { EmergencyStop(); return Connected; }
            if (!Connected || closingPort) { AddLog("Chưa kết nối cổng COM.", true); return false; }
            if (!protocolReady) { AddLog("Chưa nhận diện firmware PID PROTO=3.", true); return false; }
            if (awaitingStop) { AddLog("Đang đợi STM32 xác nhận dừng. Chưa gửi lệnh tiếp.", true); return false; }
            if (DateTime.UtcNow < calibratingUntil && cmd != "STATUS") { AddLog("Đợi hiệu chuẩn xong trước khi gửi lệnh.", true); return false; }
            if (cmd.Length > 78 || cmd.IndexOfAny(new char[] { '\r', '\n' }) >= 0) { AddLog("Chỉ gửi một lệnh, tối đa 78 ký tự.", true); return false; }
            string word = cmd.Split(' ')[0].ToUpperInvariant();
            if (Protocol.RequiresMotorOff(cmd) && (DateTime.UtcNow - lastStatus > TimeSpan.FromSeconds(2)
                || (reportedMode != "STOP" && reportedMode != "FAULT")))
            { AddLog("Lệnh đổi dấu/trục (" + word + ") chỉ gửi được khi motor ĐÃ DỪNG (bấm X trước).", true); return false; }
            if (Protocol.NeedsStopped(cmd) && (DateTime.UtcNow - lastStatus > TimeSpan.FromSeconds(2)
                || (reportedMode != "STOP" && reportedMode != "FAULT" && reportedMode != "BALANCE")))
            { AddLog("Bấm DỪNG hoặc vào BALANCE ổn định trước khi chỉnh thông số.", true); return false; }
            if (word == "M" && reportedMode == "BALANCE") { AddLog("Dừng cân bằng trước khi thử motor riêng.", true); return false; }
            try
            {
                port.Write(cmd + "\r"); AddLog("→ " + cmd, false);
                if (word == "C") { calibratingUntil = DateTime.UtcNow.AddSeconds(8); SetConnectionState(); }
                if (word == "M" || word == "ARM") { reportedMode = "PENDING"; statusRequested = true; }
                nextPoll = DateTime.UtcNow.AddMilliseconds(150); return true;
            }
            catch (Exception ex) { ConnectionLost(ex); return false; }
        }
        private void EmergencyStop()
        {
            if (!Connected) return;
            if (!protocolReady) { AddLog("Chưa nhận diện firmware nên chưa xác nhận cách dừng. Ngắt nguồn VM nếu motor đang chạy.", true); return; }
            try
            {
                port.Write("X"); awaitingStop = true; reportedMode = "STOP_PENDING";
                state.Text = "ĐANG GỬI DỪNG"; state.ForeColor = Color.Firebrick;
                AddLog("→ X — đợi STM32 xác nhận motor dừng", false); SetConnectionState();
            }
            catch (Exception ex) { ConnectionLost(ex); }
        }
        private void BeginDisconnect(bool closeForm)
        {
            closeWindow = closeForm;
            if (!Connected) { if (closeForm) Close(); return; }
            if (!protocolReady) { DropPort(); if (closeForm) Close(); return; }
            EmergencyStop(); closingPort = true; closeAt = DateTime.UtcNow.AddMilliseconds(400);
            SetConnectionState();
        }
        private void Poll(object sender, EventArgs args)
        {
            if (!Connected) return;
            try
            {
                if (port.BytesToRead > 0) reader.Push(port.ReadExisting(), OnLine);
                if (!protocolReady && DateTime.UtcNow >= handshakeDeadline)
                {
                    AddLog("Không nhận diện được firmware PID PROTO=3. Nạp Project_self_tuning mới nhất rồi kết nối lại.", true);
                    DropPort(); return;
                }
                if (closingPort && (DateTime.UtcNow >= closeAt || !awaitingStop))
                {
                    if (awaitingStop) AddLog("Chưa nhận xác nhận dừng trước khi đóng COM. Dùng ngắt nguồn VM nếu cần.", true);
                    DropPort(); if (closeWindow) { closeWindow = false; Close(); } return;
                }
                if (DateTime.UtcNow >= calibratingUntil) SetConnectionState();
                if (protocolReady && !closingPort && !awaitingStop && DateTime.UtcNow >= calibratingUntil
                    && (statusRequested || autoStatus.Checked) && DateTime.UtcNow >= nextPoll)
                { port.Write("STATUS\r"); statusRequested = false; nextPoll = DateTime.UtcNow.AddMilliseconds(150); }
                if (lastStatus != DateTime.MinValue && DateTime.UtcNow - lastStatus > TimeSpan.FromSeconds(2))
                    age.Text = "Chưa có dữ liệu mới >2s";
            }
            catch (Exception ex) { ConnectionLost(ex); }
        }
        private static float ParseFloatOrZero(Dictionary<string, string> fields, string key)
        {
            string txt; float val;
            if (fields.TryGetValue(key, out txt) && float.TryParse(txt, NumberStyles.Float, CultureInfo.InvariantCulture, out val))
                return val;
            return 0f;
        }
        private static string FormatF(string txt)
        {
            decimal v;
            string clean = (txt ?? "0").Trim().Replace(',', '.');
            if (!decimal.TryParse(clean, NumberStyles.Float, CultureInfo.InvariantCulture, out v)) return txt + "f";
            return v.ToString("0.00##", CultureInfo.InvariantCulture) + "f";
        }
        private void CopyBoardConfig()
        {
            var sb = new StringBuilder();
            sb.AppendLine("#define DEFAULT_KP              " + FormatF(kp.Text));
            sb.AppendLine("#define DEFAULT_KI              " + FormatF(ki.Text));
            sb.AppendLine("#define DEFAULT_KD              " + FormatF(kd.Text));
            sb.AppendLine("#define DEFAULT_ZERO_DEG        " + FormatF(zero.Text));
            sb.AppendLine("#define DEFAULT_DEADBAND        " + dead.Text.Trim());
            sb.AppendLine("#define MOTOR_DEADBAND_RAMP     " + ramp.Text.Trim());
            sb.AppendLine("#define DEFAULT_PWM_LIMIT       " + limit.Text.Trim());
            sb.AppendLine("#define DEFAULT_CONTROL_SIGN    " + sign.Text);
            sb.AppendLine("#define DEFAULT_LEFT_SIGN       " + dirA.Text);
            sb.AppendLine("#define DEFAULT_RIGHT_SIGN      " + dirB.Text);
            sb.AppendLine("#define DEFAULT_KP_POS          " + FormatF(pkp.Text));
            sb.AppendLine("#define DEFAULT_KI_POS          " + FormatF(pki.Text));
            sb.AppendLine("#define DEFAULT_KD_POS          " + FormatF(pkd.Text));
            sb.AppendLine("#define YAW_HOLD_KP_PULSE       " + FormatF(ykp.Text));
            sb.AppendLine("#define YAW_HOLD_KD_RATE        " + FormatF(ykd.Text));
            sb.AppendLine("#define DEFAULT_YAW_SIGN        (" + ysign.Text + ")");
            sb.AppendLine("#define DEFAULT_ENC_LEFT_SIGN   " + edirL.Text);
            sb.AppendLine("#define DEFAULT_ENC_RIGHT_SIGN  " + edirR.Text);
            string snippet = sb.ToString();
            try { Clipboard.SetText(snippet); AddLog("Đã copy macro board_config.h vào Clipboard:\r\n" + snippet.TrimEnd(), false); }
            catch (Exception ex) { AddLog("Không truy cập được Clipboard (" + ex.Message + "). Nội dung:\r\n" + snippet.TrimEnd(), false); }
        }
        private void OnLine(string line)
        {
            if (Protocol.IsPidGreeting(line))
            {
                protocolReady = true; statusRequested = true; firmwareVersion = "FW v4"; nextPoll = DateTime.UtcNow.AddMilliseconds(80);
                if (lastStatus == DateTime.MinValue) state.Text = "ĐÃ KẾT NỐI · ĐỢI TRẠNG THÁI";
                SetConnectionState();
            }
            if (line.StartsWith("CALIB:", StringComparison.Ordinal))
            { calibratingUntil = DateTime.UtcNow.AddSeconds(8); SetConnectionState(); }
            var fields = Protocol.Fields(line); string value;
            if (fields.TryGetValue("MODE", out value))
            {
                if (!Protocol.IsStatus(line, fields))
                {
                    statusRequested = true;
                    AddLog("Bỏ qua dòng trạng thái lỗi; đang chờ số hợp lệ từ STM32.", true);
                    return;
                }
                reportedMode = value; lastStatus = DateTime.UtcNow; state.Text = "Chế độ: " + value;
                state.ForeColor = value == "FAULT" ? Color.Firebrick : (value == "STOP" ? Color.DarkGreen : Color.DarkBlue);
                float angVal = ParseFloatOrZero(fields, "ANG");
                float zeroVal = 0f;
                float.TryParse((zero.Text ?? "0").Replace(',', '.'), NumberStyles.Float, CultureInfo.InvariantCulture, out zeroVal);
                float trefVal = ParseFloatOrZero(fields, "TREF");
                float pwmA = ParseFloatOrZero(fields, "A"), pwmB = ParseFloatOrZero(fields, "B");
                if (fields.TryGetValue("ANG", out value)) angle.Text = "Góc: " + value + "°";
                if (fields.TryGetValue("IMU", out value)) imu.Text = "IMU: " + value + " (" + lastWho + ")";
                string gzTxt = fields.TryGetValue("GZ", out value) ? value : "0.00";
                if (fields.TryGetValue("RATE", out value)) rate.Text = "Gyro P/Z: " + value + " / " + gzTxt + " °/s";
                string turnTxt = fields.TryGetValue("TURN", out value) ? " · Turn=" + value : "";
                pwm.Text = "PWM A: " + fields["A"] + " / B: " + fields["B"] + turnTxt + (fields.ContainsKey("ENC") ? " · ENC: " + fields["ENC"] : "");
                if (fields.TryGetValue("REASON", out value))
                {
                    string posInfo = fields.ContainsKey("X")
                        ? " · X=" + fields["X"] + "cm V=" + (fields.ContainsKey("V") ? fields["V"] : "0") + " (θref=" + (fields.ContainsKey("TREF") ? fields["TREF"] : "0") + "°)"
                        : "";
                    reason.Text = "Lý do: " + value + posInfo;
                }
                age.Text = "Dữ liệu: " + DateTime.Now.ToString("HH:mm:ss") + (fields.ContainsKey("AGE") ? " (" + fields["AGE"] + "ms)" : "");
                chart.AddSample(angVal - zeroVal, trefVal, (pwmA + pwmB) * 0.5f / 50f);
                if (logTelemetry.Checked) AddLog("← " + line, false);
                return;
            }
            if (fields.TryGetValue("PID", out value))
            {
                actualPid.Text = "STM32 PID: " + value;
                if (fields.ContainsKey("WHO")) lastWho = fields["WHO"];
                string ppidTxt = fields.ContainsKey("PPID") ? fields["PPID"] : "—";
                string ypidTxt = fields.ContainsKey("YPID") ? fields["YPID"] : "—";
                actualOuter.Text = "PPID: " + ppidTxt + " | YPID: " + ypidTxt;
                if (fields.ContainsKey("TESTMAX")) firmwareVersion = "FW v4 · M ±" + fields["TESTMAX"];
                SetConnectionState();
                if (applyNextSettings)
                {
                    string[] values = value.Split(',');
                    if (values.Length == 3) { kp.Text = values[0]; ki.Text = values[1]; kd.Text = values[2]; }
                    if (fields.ContainsKey("PPID"))
                    {
                        string[] pv = fields["PPID"].Split(',');
                        if (pv.Length == 3) { pkp.Text = pv[0]; pki.Text = pv[1]; pkd.Text = pv[2]; }
                    }
                    if (fields.ContainsKey("YPID"))
                    {
                        string[] yv = fields["YPID"].Split(',');
                        if (yv.Length == 2) { ykp.Text = yv[0]; ykd.Text = yv[1]; }
                    }
                    if (fields.ContainsKey("ZERO")) zero.Text = fields["ZERO"];
                    if (fields.ContainsKey("LIMIT")) limit.Text = fields["LIMIT"];
                    if (fields.ContainsKey("DEAD")) dead.Text = fields["DEAD"];
                    if (fields.ContainsKey("RAMP")) ramp.Text = fields["RAMP"];
                    if (fields.ContainsKey("DIR")) { string[] dirs = fields["DIR"].Split(','); if (dirs.Length == 2) { dirA.Text = dirs[0]; dirB.Text = dirs[1]; } }
                    if (fields.ContainsKey("SIGN")) sign.Text = fields["SIGN"];
                    if (fields.ContainsKey("EDIR")) { string[] edirs = fields["EDIR"].Split(','); if (edirs.Length == 2) { edirL.Text = edirs[0]; edirR.Text = edirs[1]; } }
                    if (fields.ContainsKey("YSIGN")) ysign.Text = fields["YSIGN"];
                    applyNextSettings = false; AddLog("Đã đọc đầy đủ thông số từ STM32 vào các ô nhập.", false);
                }
                return;
            }
            if (line.StartsWith("MOTOR STOPPED") || line.StartsWith("STOP: STBY"))
            { awaitingStop = false; statusRequested = true; reportedMode = "STOP"; state.Text = "ĐÃ DỪNG"; state.ForeColor = Color.DarkGreen; nextPoll = DateTime.UtcNow; SetConnectionState(); }
            if (line.StartsWith("CALIB OK") || line.StartsWith("CALIB FAILED"))
            { calibratingUntil = DateTime.MinValue; statusRequested = true; nextPoll = DateTime.UtcNow; SetConnectionState(); }
            if (line.StartsWith("EVENT MOTOR_OFF", StringComparison.Ordinal))
            { statusRequested = true; nextPoll = DateTime.UtcNow; }
            AddLog("← " + line, line.StartsWith("ERR") || line.IndexOf("FAILED", StringComparison.Ordinal) >= 0);
        }
        private void DropPort()
        {
            try { if (port != null) port.Dispose(); } catch { }
            port = null; awaitingStop = false; closingPort = false; protocolReady = false; calibratingUntil = DateTime.MinValue;
            applyNextSettings = false; statusRequested = false;
            state.Text = "CHƯA KẾT NỐI"; state.ForeColor = Color.Gray; SetConnectionState();
        }
        private void ConnectionLost(Exception ex)
        {
            DropPort(); AddLog("Lỗi kết nối: " + ex.Message + ". Chưa xác nhận được motor dừng; dùng ngắt nguồn VM nếu cần.", true);
        }
        private void AddLog(string text, bool error)
        {
            if (log.TextLength > 45000) { log.Select(0, 15000); log.SelectedText = ""; }
            log.SelectionStart = log.TextLength;
            log.AppendText(DateTime.Now.ToString("HH:mm:ss") + (error ? " [LỖI] " : " ") + text + Environment.NewLine); log.ScrollToCaret();
        }
        private void SaveLog()
        {
            using (var dialog = new SaveFileDialog { Filter = "Text log (*.txt)|*.txt", FileName = "PID_log_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".txt" })
                if (dialog.ShowDialog(this) == DialogResult.OK) File.WriteAllText(dialog.FileName, log.Text, Encoding.UTF8);
        }
        private void OnClosing(object sender, FormClosingEventArgs e)
        {
            if (Connected) { e.Cancel = true; if (!closingPort) BeginDisconnect(true); }
            else timer.Stop();
        }
        public void RenderPreview(string filename)
        {
            preview = true;
            ShowInTaskbar = false; StartPosition = FormStartPosition.Manual;
            Location = new Point(-20000, -20000);
            Show(); Application.DoEvents();
            for (int i = 0; i < 40; i++)
            {
                float a = 1.50f + (float)Math.Sin(i * 0.25) * 3.5f;
                float tr = (float)Math.Cos(i * 0.2) * 1.2f;
                int pw = (int)(Math.Sin(i * 0.25) * 180);
                OnLine(string.Format(CultureInfo.InvariantCulture,
                    "MODE=BALANCE IMU=OK ANG={0:0.00} RATE={1:0.00} GZ=0.12 A={2} B={3} TURN=4 ENC=12,-14 X=0.45 V=1.20 TREF={4:0.00} REASON=BALANCE N={5} AGE=2",
                    a, Math.Cos(i * 0.25) * 8.0, pw, pw, tr, 100 + i));
            }
            OnLine("PID=92.0000,0.0000,5.5000 ZERO=1.5000 LIMIT=999 DEAD=110 RAMP=48 DIR=-1,-1 SIGN=-1 AXIS=Y PPID=9.0000,1.8000,12.5000 YPID=0.3000,1.2000 YSIGN=-1 CPR=1400.00 EDIR=1,-1 ENC=1 ADDR=0x68 WHO=0x12 TESTMAX=999");
            SetConnectionState();
            connection.Text = "BẢN XEM TRƯỚC v4.2 (VM 6.0V CAD) — chưa mở cổng COM, chưa kết nối thiết bị.";
            PerformLayout(); Application.DoEvents();
            using (var bitmap = new Bitmap(Width, Height)) { DrawToBitmap(bitmap, new Rectangle(0, 0, Width, Height)); bitmap.Save(filename); }
            Hide();
        }
        public string UiSelfTest()
        {
            kp.Text = "17.3"; ki.Text = "0.1"; kd.Text = "0.2";
            pkp.Text = "9.9"; raw.Text = "PID 17.3 0.1 0.2";
            OnLine("MODE=STOP IMU=OK ANG=1.29 RATE=0.03 GZ=-0.10 A=0 B=0 TURN=0 ENC=0,0 X=0.00 V=0.00 TREF=0.00 REASON=READY N=100 AGE=2");
            OnLine("PID=0.00,0.00,0.00 ZERO=0 LIMIT=300 DEAD=0 RAMP=15 DIR=1,1 SIGN=1 PPID=1.0,0.0,1.0 YPID=0.1,0.2 EDIR=1,1 YSIGN=1 WHO=0x12 TESTMAX=999");
            if (kp.Text != "17.3" || ki.Text != "0.1" || kd.Text != "0.2" || pkp.Text != "9.9" || raw.Text != "PID 17.3 0.1 0.2")
                throw new Exception("Incoming telemetry overwrote command inputs");
            applyNextSettings = true;
            OnLine("PID=92.00,0.00,5.50 ZERO=1.50 LIMIT=999 DEAD=110 RAMP=48 DIR=-1,-1 SIGN=-1 PPID=9.00,1.80,12.50 YPID=0.30,1.20 EDIR=1,-1 YSIGN=-1 WHO=0x12 TESTMAX=999");
            if (kp.Text != "92.00" || pkp.Text != "9.00" || ykp.Text != "0.30" || zero.Text != "1.50" || dead.Text != "110" || ramp.Text != "48" || dirA.Text != "-1" || edirR.Text != "-1" || ysign.Text != "-1" || lastWho != "0x12")
                throw new Exception("Explicit settings read failed");
            string beforeAngle = angle.Text, beforeMode = reportedMode;
            OnLine("MODE=FAULT IMU=CALIB_MOVING ANG=0PID_PANEL RATE=0 A=0 B=0 REASON=CALIB_MOVING");
            if (angle.Text != beforeAngle || reportedMode != beforeMode || !statusRequested)
                throw new Exception("Malformed angle changed displayed state");
            autoStatus.Checked = false; statusRequested = false;
            OnLine("CALIB OK. Motors OFF. Send ARM to balance, X to stop.");
            if (!statusRequested || calibratingUntil != DateTime.MinValue)
                throw new Exception("Calibration did not request fresh status with auto-read disabled");
            statusRequested = false;
            OnLine("EVENT MOTOR_OFF REASON=TEST_TIMEOUT");
            if (!statusRequested) throw new Exception("Motor timeout did not request fresh status");
            ApplyPreset("Chuẩn 6V", "92.00", "0.00", "5.50", "9.00", "1.80", "12.50", "110", "48");
            if (kp.Text != "92.00" || pkd.Text != "12.50" || ramp.Text != "48")
                throw new Exception("Preset application failed");
            if (Connected) throw new Exception("UI test must not open a COM port");
            return "PASS: v4.2 inputs preserved; 6V Presets/PPID/YPID/DEAD/RAMP/EDIR/YSIGN verified; malformed angle rejected. No COM port opened.";
        }
    }

    internal static class SelfTest
    {
        private static void Check(bool value, string name) { if (!value) throw new Exception("FAIL: " + name); }
        private static bool Reject(Action action) { try { action(); return false; } catch (FormatException) { return true; } }
        public static string Run()
        {
            Check(Protocol.Pid("78", "0", "5,2") == "PID 78 0 5.2", "PID decimal comma");
            Check(Protocol.PosPid("8.5", "8.5", "11.5") == "PPID 8.5 8.5 11.5", "PPID Ki<=10 format");
            Check(Reject(delegate { Protocol.PosPid("8.5", "10.5", "11.5"); }), "PPID Ki>10 reject");
            Check(Protocol.YawPid("0.45", "1.1") == "YPID 0.45 1.1", "YPID format");
            Check(Reject(delegate { Protocol.Pid("NaN", "0", "0"); }), "reject NaN");
            Check(Reject(delegate { Protocol.Pid("201", "0", "0"); }), "PID range");
            Check(Reject(delegate { Protocol.YawPid("6", "0"); }), "YPID range");
            Check(Protocol.Motor(500, -999) == "M 500 -999", "motor command");
            Check(Reject(delegate { Protocol.Motor(1000, 0); }), "motor range");
            Check(Reject(delegate { Protocol.Motor(100.5m, 0); }), "motor integer");
            Check(Protocol.NeedsStopped("PID 5 0 0") && Protocol.NeedsStopped("YPID 0.5 1.1") && !Protocol.NeedsStopped("STATUS"), "settings gate");
            Check(Protocol.RequiresMotorOff("DIR -1 -1") && Protocol.RequiresMotorOff("YSIGN -1") && !Protocol.RequiresMotorOff("PPID 8.5 1.4 11.5"), "motor-off gate");
            Check(Protocol.IsPidGreeting("PID_PANEL PROTO=3 FW=Project_self_tuning-v4.0"), "PID v3 handshake");
            Check(!Protocol.IsPidGreeting("Project_self_tuning: old firmware"), "reject incompatible PID firmware");
            Check(!Protocol.IsPidGreeting("PID_PANEL PROTO=30 FW=unknown"), "reject different protocol revision");
            var values = Protocol.Fields("MODE=STOP IMU=OK ANG=1.50 GZ=0.12 A=0 B=0 TURN=0 REASON=TEST_TIMEOUT");
            Check(values["ANG"] == "1.50" && values["MODE"] == "STOP" && values["GZ"] == "0.12", "status parse");
            string frame = "MODE=STOP IMU=OK ANG=1.50 RATE=0.03 GZ=0.00 A=0 B=0 TURN=0 REASON=READY";
            Check(Protocol.IsStatus(frame, Protocol.Fields(frame)), "valid status");
            foreach (string bad in new string[] { frame.Replace("1.50", "0PID_PANEL"), frame.Replace("1.50", "NaN"), frame.Replace("MODE=STOP", "MODE=BROKEN"), "MODE=STOP IMU=OK ANG=0" })
                Check(!Protocol.IsStatus(bad, Protocol.Fields(bad)), "reject malformed status");
            var list = new List<string>(); var parser = new LineReader();
            parser.Push("MODE=ST", list.Add); parser.Push("OP\r\nPID=78,0,5.2\n", list.Add);
            Check(list.Count == 2 && list[0] == "MODE=STOP", "partial serial frames");
            parser.Push(new String('a', 3000) + "\r\nOK\r\n", list.Add);
            Check(list.Count == 3 && list[2] == "OK", "bounded serial lines");
            return "PASS: v4.1 PID/PPID/YPID/RAMP checks, NaN/range, motor commands, gates, telemetry parsing, chart framing. No COM port opened.";
        }
    }

    internal static class Program
    {
        [STAThread]
        private static int Main(string[] args)
        {
            try
            {
                if (args.Length > 0 && args[0] == "--self-test")
                {
                    string result = SelfTest.Run();
                    if (args.Length > 1) File.WriteAllText(args[1], result + Environment.NewLine, Encoding.UTF8);
                    return 0;
                }
                Application.EnableVisualStyles(); Application.SetCompatibleTextRenderingDefault(false);
                using (var form = new PanelForm())
                {
                    if (args.Length == 2 && args[0] == "--ui-test") { File.WriteAllText(args[1], form.UiSelfTest(), Encoding.UTF8); return 0; }
                    if (args.Length == 2 && args[0] == "--render-preview") { form.RenderPreview(args[1]); return 0; }
                    Application.Run(form);
                }
                return 0;
            }
            catch (Exception ex)
            {
                if (args.Length > 1 && args[0] == "--self-test") File.WriteAllText(args[1], ex.ToString(), Encoding.UTF8);
                else MessageBox.Show(ex.Message, "Motor PID Panel", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return 1;
            }
        }
    }
}
