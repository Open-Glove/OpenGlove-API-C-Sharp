using System;
using System.Globalization;
using System.IO.Ports;
using System.Text;

namespace OpenGlove
{
    /// <summary>
    /// Comunicación serial con el guante. Soporta trama LSM9DS1 (EOF) y BNO055 (8 floats dual quat).
    /// Emite eventos tipados; ReadLine/Write se mantienen por compatibilidad.
    /// </summary>
    public class Communication
    {
        public enum ImuParseMode
        {
            Bno055,
            Lsm9ds1
        }

        public delegate void ImuValuesHandler(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
        public delegate void QuaternionHandler(float qx, float qy, float qz);
        public delegate void DualQuaternionHandler(float w1, float x1, float y1, float z1, float w2, float x2, float y2, float z2);
        public delegate void LineHandler(string line);

        private SerialPort port = new SerialPort();
        private readonly object _sbLock = new object();
        private readonly StringBuilder _sb = new StringBuilder();
        private readonly CultureInfo _inv = CultureInfo.InvariantCulture;
        private ImuParseMode _imuMode = ImuParseMode.Bno055;

        public event ImuValuesHandler imu_ValuesFunction;
        public event QuaternionHandler quaternionFunction;
        public event DualQuaternionHandler dualQuaternionFunction;
        /// <summary>Líneas no-IMU (p. ej. flexores) o reenvío crudo.</summary>
        public event LineHandler lineReceived;

        public Communication()
        {
        }

        public Communication(string portName, int baudRate)
        {
            this.port.PortName = portName;
            this.port.BaudRate = baudRate;
            this.port.Open();
        }

        public string[] GetPortNames()
        {
            return SerialPort.GetPortNames();
        }

        public void SetImuModel(string imuModel)
        {
            if (string.Equals(imuModel, "LSM9DS1", StringComparison.OrdinalIgnoreCase)
                || string.Equals(imuModel, "Default", StringComparison.OrdinalIgnoreCase))
            {
                _imuMode = ImuParseMode.Lsm9ds1;
            }
            else
            {
                _imuMode = ImuParseMode.Bno055;
            }
        }

        public void OpenPort(string portName, int baudRate)
        {
            this.port.PortName = portName;
            this.port.BaudRate = baudRate;
            this.port.NewLine = "\n";
            this.port.ReadTimeout = 2000;
            this.port.DtrEnable = true;
            this.port.RtsEnable = true;
            this.port.Handshake = Handshake.None;
            this.port.DataReceived -= SerialPort_DataReceived;
            this.port.DataReceived += SerialPort_DataReceived;
            this.port.Open();
        }

        public void Write(string data)
        {
            this.port.Write(data);
        }

        public string ReadLine()
        {
            return this.port.ReadLine();
        }

        public void ClosePort()
        {
            try
            {
                this.port.DataReceived -= SerialPort_DataReceived;
                if (this.port.IsOpen)
                    this.port.Close();
            }
            catch
            {
            }
        }

        private void RaiseLine(string line)
        {
            LineHandler handler = lineReceived;
            if (handler != null)
                handler(line);
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort currentSerialPort = (SerialPort)sender;
            try
            {
                string chunk = currentSerialPort.ReadExisting();
                if (string.IsNullOrEmpty(chunk))
                    return;

                lock (_sbLock)
                {
                    _sb.Append(chunk);

                    while (true)
                    {
                        string text = _sb.ToString();
                        int nl = text.IndexOf('\n');
                        if (nl < 0)
                            break;

                        string line = text.Substring(0, nl).Trim('\r', ' ', '\t');
                        _sb.Remove(0, nl + 1);

                        if (string.IsNullOrEmpty(line))
                            continue;

                        float w1, x1, y1, z1, w2, x2, y2, z2;
                        if (_imuMode == ImuParseMode.Bno055
                            && TryParseDualQuaternionLine(line, out w1, out x1, out y1, out z1, out w2, out x2, out y2, out z2))
                        {
                            DualQuaternionHandler dq = dualQuaternionFunction;
                            if (dq != null)
                                dq(w1, x1, y1, z1, w2, x2, y2, z2);
                            RaiseLine("Q," + w1.ToString(_inv) + "," + x1.ToString(_inv) + "," + y1.ToString(_inv) + "," + z1.ToString(_inv)
                                + "," + w2.ToString(_inv) + "," + x2.ToString(_inv) + "," + y2.ToString(_inv) + "," + z2.ToString(_inv));
                        }
                        else
                        {
                            RaiseLine(line);
                        }
                    }

                    if (_imuMode == ImuParseMode.Lsm9ds1)
                    {
                        while (true)
                        {
                            string text = _sb.ToString();
                            int eof = text.IndexOf("EOF");
                            if (eof < 0)
                                break;

                            string frame = text.Substring(0, eof);
                            _sb.Remove(0, eof + 3);
                            ProcessImuFrame(frame);
                        }
                    }
                }
            }
            catch
            {
            }
        }

        private bool TryParseDualQuaternionLine(string line, out float w1, out float x1, out float y1, out float z1, out float w2, out float x2, out float y2, out float z2)
        {
            w1 = x1 = y1 = z1 = w2 = x2 = y2 = z2 = 0f;
            string[] parts = line.Split(',');
            if (parts.Length != 8)
                return false;

            if (!float.TryParse(parts[0], NumberStyles.Float, _inv, out w1)) return false;
            if (!float.TryParse(parts[1], NumberStyles.Float, _inv, out x1)) return false;
            if (!float.TryParse(parts[2], NumberStyles.Float, _inv, out y1)) return false;
            if (!float.TryParse(parts[3], NumberStyles.Float, _inv, out z1)) return false;
            if (!float.TryParse(parts[4], NumberStyles.Float, _inv, out w2)) return false;
            if (!float.TryParse(parts[5], NumberStyles.Float, _inv, out x2)) return false;
            if (!float.TryParse(parts[6], NumberStyles.Float, _inv, out y2)) return false;
            if (!float.TryParse(parts[7], NumberStyles.Float, _inv, out z2)) return false;
            return true;
        }

        private void ProcessImuFrame(string frame)
        {
            string[] parts = frame.Split('/');
            if (parts.Length != 12)
                return;

            double axD, ayD, azD, gxD, gyD, gzD, mxD, myD, mzD, qxD, qyD, qzD;
            if (!double.TryParse(parts[0], NumberStyles.Float, _inv, out axD)) return;
            if (!double.TryParse(parts[1], NumberStyles.Float, _inv, out ayD)) return;
            if (!double.TryParse(parts[2], NumberStyles.Float, _inv, out azD)) return;
            if (!double.TryParse(parts[3], NumberStyles.Float, _inv, out gxD)) return;
            if (!double.TryParse(parts[4], NumberStyles.Float, _inv, out gyD)) return;
            if (!double.TryParse(parts[5], NumberStyles.Float, _inv, out gzD)) return;
            if (!double.TryParse(parts[6], NumberStyles.Float, _inv, out mxD)) return;
            if (!double.TryParse(parts[7], NumberStyles.Float, _inv, out myD)) return;
            if (!double.TryParse(parts[8], NumberStyles.Float, _inv, out mzD)) return;
            if (!double.TryParse(parts[9], NumberStyles.Float, _inv, out qxD)) return;
            if (!double.TryParse(parts[10], NumberStyles.Float, _inv, out qyD)) return;
            if (!double.TryParse(parts[11], NumberStyles.Float, _inv, out qzD)) return;

            float ax = (float)axD, ay = (float)ayD, az = (float)azD;
            float gx = (float)gxD, gy = (float)gyD, gz = (float)gzD;
            float mx = (float)mxD, my = (float)myD, mz = (float)mzD;
            float qx = (float)qxD, qy = (float)qyD, qz = (float)qzD;

            ImuValuesHandler imu = imu_ValuesFunction;
            if (imu != null)
                imu(ax, ay, az, gx, gy, gz, mx, my, mz);

            QuaternionHandler qh = quaternionFunction;
            if (qh != null)
                qh(qx, qy, qz);

            RaiseLine("z," + ax.ToString(_inv) + "," + ay.ToString(_inv) + "," + az.ToString(_inv)
                + "," + gx.ToString(_inv) + "," + gy.ToString(_inv) + "," + gz.ToString(_inv)
                + "," + mx.ToString(_inv) + "," + my.ToString(_inv) + "," + mz.ToString(_inv));
            RaiseLine("q," + qx.ToString(_inv) + "," + qy.ToString(_inv) + "," + qz.ToString(_inv));
        }
    }
}
