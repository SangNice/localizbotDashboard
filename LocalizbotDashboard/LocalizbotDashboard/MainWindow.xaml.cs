using System;
using System.IO;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;

namespace LocalizbotDashboard
{
    public partial class MainWindow : Window
    {
        private ClientWebSocket? _ws;
        private CancellationTokenSource? _cts;

        // Topic điều khiển vận tốc:
        // Nếu dùng diff_drive_controller của ros2_control (Jazzy/Humble), thường là "/diff_cont/cmd_vel_unstamped" hoặc "/cmd_vel"
        private const string CMD_VEL_TOPIC = "/diff_cont/cmd_vel";
        // Topic Odometry sau bộ lọc EKF (hoặc đổi thành "/diff_cont/odom" nếu chưa bật ekf.launch.py)
        private const string ODOM_TOPIC = "/odometry/filtered";

        // Thông số bản đồ SLAM hiện tại để quy đổi tọa độ (m) -> Pixel trên Canvas
        private double _mapResolution = 0.05; // m/pixel mặc định
        private int _mapHeight = 0;
        private const double WORLD_CENTER = 1000.0; // Tâm của WorldCanvas (2000x2000)

        // Biến điều khiển Pan/Zoom bản đồ bằng chuột
        private bool _isPanning = false;
        private Point _lastMousePos;

        // Biến giữ lệnh vận tốc hiện tại để phát định kỳ 10Hz khi nhấn giữ nút
        private double _cmdLinearX = 0.0;
        private double _cmdAngularZ = 0.0;
        private bool _isSendingCmd = false;
        private readonly DispatcherTimer _teleopTimer;

        public MainWindow()
        {
            InitializeComponent();

            // Timer phát lệnh điều khiển động cơ ở tần số 10Hz (100ms)
            _teleopTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(100)
            };
            _teleopTimer.Tick += async (s, e) =>
            {
                if (_isSendingCmd)
                {
                    await PublishCmdVelAsync(_cmdLinearX, _cmdAngularZ);
                }
            };
            _teleopTimer.Start();
        }

        // =========================================================================
        // 1. KẾT NỐI WEBSOCKET VỚI ROSBRIDGE_SERVER & ĐĂNG KÝ TOPIC (SUBSCRIBE)
        // =========================================================================
        private async void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            if (_ws != null && _ws.State == WebSocketState.Open)
            {
                await DisconnectRosAsync();
                return;
            }

            string url = TxtRosUrl.Text.Trim();
            try
            {
                BtnConnect.IsEnabled = false;
                TxtConnStatus.Text = "● Trạng thái: Đang kết nối...";
                TxtConnStatus.Foreground = new SolidColorBrush(Colors.Orange);

                _ws = new ClientWebSocket();
                _cts = new CancellationTokenSource();
                await _ws.ConnectAsync(new Uri(url), _cts.Token);

                TxtConnStatus.Text = $"● Đã kết nối: {url}";
                TxtConnStatus.Foreground = new SolidColorBrush(Color.FromRgb(74, 222, 128));
                BtnConnect.Content = "Ngắt kết nối";
                BtnConnect.Background = new SolidColorBrush(Color.FromRgb(220, 38, 38));
                BtnConnect.IsEnabled = true;

                // Đăng ký nhận các Topic từ ROS 2
                await SubscribeTopicAsync("/map", "nav_msgs/msg/OccupancyGrid", 7000);
                await SubscribeTopicAsync(ODOM_TOPIC, "nav_msgs/msg/Odometry", 50);
                await SubscribeTopicAsync("/diff_cont/odom", "nav_msgs/msg/Odometry", 50); // Dự phòng nếu chưa bật EKF
                await SubscribeTopicAsync("/imu/data", "sensor_msgs/msg/Imu", 100);
                await SubscribeTopicAsync("/ultrasonic/range", "sensor_msgs/msg/Range", 100);

                // Quảng bá (Advertise) topic điều khiển vận tốc
                await SendJsonAsync(new
                {
                    op = "advertise",
                    topic = CMD_VEL_TOPIC,
                    type = "geometry_msgs/msg/TwistStamped"
                });

                // Bắt đầu luồng lắng nghe dữ liệu từ ROS 2
                _ = Task.Run(() => ReceiveLoopAsync(_cts.Token));
            }
            catch (Exception ex)
            {
                TxtConnStatus.Text = "● Lỗi kết nối: Kiểm tra rosbridge_server!";
                TxtConnStatus.Foreground = new SolidColorBrush(Color.FromRgb(239, 68, 68));
                BtnConnect.IsEnabled = true;
                MessageBox.Show($"Không thể kết nối tới {url}\nChi tiết: {ex.Message}", "Lỗi Kết Nối ROS 2", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private async Task SubscribeTopicAsync(string topic, string msgType, int throttleRateMs)
        {
            await SendJsonAsync(new
            {
                op = "subscribe",
                topic = topic,
                type = msgType,
                throttle_rate = throttleRateMs,
                queue_length = 1
            });
        }

        private async Task DisconnectRosAsync()
        {
            try
            {
                _isSendingCmd = false;
                await PublishCmdVelAsync(0, 0);
                _cts?.Cancel();
                if (_ws != null && _ws.State == WebSocketState.Open)
                {
                    await _ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "User Disconnected", CancellationToken.None);
                }
            }
            catch { }
            finally
            {
                _ws?.Dispose();
                _ws = null;
                TxtConnStatus.Text = "● Trạng thái: Đã ngắt kết nối";
                TxtConnStatus.Foreground = new SolidColorBrush(Color.FromRgb(239, 68, 68));
                BtnConnect.Content = "Kết nối";
                BtnConnect.Background = new SolidColorBrush(Color.FromRgb(22, 163, 74));
                BtnConnect.IsEnabled = true;
            }
        }

        private async Task SendJsonAsync(object payload)
        {
            if (_ws == null || _ws.State != WebSocketState.Open) return;
            string json = JsonSerializer.Serialize(payload);
            byte[] bytes = Encoding.UTF8.GetBytes(json);
            await _ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, CancellationToken.None);
        }

        // =========================================================================
        // 2. VÒNG LẶP NHẬN & GIẢI MÃ DỮ LIỆU TỪ ROS 2 (MAP, ODOM, IMU, HC-SR04)
        // =========================================================================
        private async Task ReceiveLoopAsync(CancellationToken token)
        {
            byte[] buffer = new byte[65536];
            using var ms = new MemoryStream();

            while (!token.IsCancellationRequested && _ws != null && _ws.State == WebSocketState.Open)
            {
                try
                {
                    ms.SetLength(0);
                    WebSocketReceiveResult result;
                    do
                    {
                        result = await _ws.ReceiveAsync(new ArraySegment<byte>(buffer), token);
                        if (result.MessageType == WebSocketMessageType.Close) return;
                        ms.Write(buffer, 0, result.Count);
                    } while (!result.EndOfMessage);

                    string message = Encoding.UTF8.GetString(ms.ToArray());
                    ProcessRosMessage(message);
                }
                catch (OperationCanceledException) { break; }
                catch { }
            }
        }

        private void ProcessRosMessage(string jsonStr)
        {
            try
            {
                using JsonDocument doc = JsonDocument.Parse(jsonStr);
                JsonElement root = doc.RootElement;

                if (!root.TryGetProperty("op", out JsonElement opElem) || opElem.GetString() != "publish")
                    return;

                string topic = root.GetProperty("topic").GetString() ?? "";
                JsonElement msg = root.GetProperty("msg");

                if (topic == "/map")
                {
                    HandleMapMessage(msg);
                }
                else if (topic == ODOM_TOPIC || topic == "/diff_cont/odom")
                {
                    HandleOdomMessage(msg);
                }
                else if (topic == "/imu/data")
                {
                    double gz = msg.GetProperty("angular_velocity").GetProperty("z").GetDouble();
                    Dispatcher.InvokeAsync(() => TxtImuGz.Text = gz.ToString("F3"));
                }
                else if (topic == "/ultrasonic/range")
                {
                    double rangeM = msg.GetProperty("range").GetDouble();
                    Dispatcher.InvokeAsync(() =>
                    {
                        TxtUltrasonic.Text = $"{rangeM:F2} m ({rangeM * 100:F0} cm)";
                        TxtUltrasonic.Foreground = rangeM < 0.25
                            ? new SolidColorBrush(Color.FromRgb(239, 68, 68)) // Đỏ nếu vật cản < 25cm
                            : new SolidColorBrush(Color.FromRgb(251, 146, 60));
                    });
                }
            }
            catch { }
        }

        // =========================================================================
        // 3. DỰNG BẢN ĐỒ SLAM (/map) LÊN CANVAS GIỐNG HỆT RVIZ2
        // =========================================================================
        private void HandleMapMessage(JsonElement msg)
        {
            JsonElement info = msg.GetProperty("info");
            int width = info.GetProperty("width").GetInt32();
            int height = info.GetProperty("height").GetInt32();
            double resolution = info.GetProperty("resolution").GetDouble();

            JsonElement originPos = info.GetProperty("origin").GetProperty("position");
            double originX = originPos.GetProperty("x").GetDouble();
            double originY = originPos.GetProperty("y").GetDouble();

            JsonElement dataArray = msg.GetProperty("data");
            if (width <= 0 || height <= 0) return;

            // Tạo mảng pixel BGRA32 để vẽ bản đồ
            byte[] pixels = new byte[width * height * 4];
            int idx = 0;

            foreach (JsonElement cell in dataArray.EnumerateArray())
            {
                int val = cell.GetInt32();
                int mapX = idx % width;
                int mapY = idx / width;

                // Lật trục Y (Vì ROS 2 gốc ở góc dưới bên trái, còn WPF gốc ở góc trên bên trái)
                int flippedY = (height - 1) - mapY;
                int pixelIndex = (flippedY * width + mapX) * 4;

                byte r, g, b;
                if (val == -1)
                {
                    // Chưa khám phá (Unknown) -> Màu xám tối kiểu RViz Dark
                    r = 39; g = 39; b = 42;
                }
                else if (val == 0)
                {
                    // Vùng trống (Free space) -> Màu xám sáng
                    r = 212; g = 212; b = 216;
                }
                else if (val >= 65)
                {
                    // Tường / Vật cản (Occupied) -> Màu đen đậm
                    r = 9; g = 9; b = 11;
                }
                else
                {
                    // Mức độ chiếm chỗ trung gian
                    byte shade = (byte)(210 - (val * 2));
                    r = shade; g = shade; b = shade;
                }

                pixels[pixelIndex + 0] = b; // Blue
                pixels[pixelIndex + 1] = g; // Green
                pixels[pixelIndex + 2] = r; // Red
                pixels[pixelIndex + 3] = 255; // Alpha
                idx++;
            }

            Dispatcher.InvokeAsync(() =>
            {
                _mapResolution = resolution;
                _mapHeight = height;

                var bitmap = new WriteableBitmap(width, height, 96, 96, PixelFormats.Bgra32, null);
                bitmap.WritePixels(new Int32Rect(0, 0, width, height), pixels, width * 4, 0);
                SlamMapImage.Source = bitmap;
                SlamMapImage.Width = width;
                SlamMapImage.Height = height;

                // Đặt vị trí ảnh bản đồ khớp với gốc tọa độ (WORLD_CENTER, WORLD_CENTER)
                double leftPx = WORLD_CENTER + (originX / resolution);
                double topPx = WORLD_CENTER - (originY / resolution) - height;

                Canvas.SetLeft(SlamMapImage, leftPx);
                Canvas.SetTop(SlamMapImage, topPx);

                TxtMapInfo.Text = $"Kích thước: {width}x{height} px | Độ phân giải: {resolution:F3} m/px";
            });
        }

        // =========================================================================
        // 4. CẬP NHẬT VỊ TRÍ & HƯỚNG CỦA ROBOT TRÊN BẢN ĐỒ TỪ ODOMETRY / EKF
        // =========================================================================
        private void HandleOdomMessage(JsonElement msg)
        {
            JsonElement pose = msg.GetProperty("pose").GetProperty("pose");
            JsonElement pos = pose.GetProperty("position");
            JsonElement ori = pose.GetProperty("orientation");
            JsonElement twist = msg.GetProperty("twist").GetProperty("twist");

            double x = pos.GetProperty("x").GetDouble();
            double y = pos.GetProperty("y").GetDouble();

            double qx = ori.GetProperty("x").GetDouble();
            double qy = ori.GetProperty("y").GetDouble();
            double qz = ori.GetProperty("z").GetDouble();
            double qw = ori.GetProperty("w").GetDouble();

            // Chuyển Quaternion sang góc Yaw (radian & độ)
            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            double yawRad = Math.Atan2(siny_cosp, cosy_cosp);
            double yawDeg = yawRad * (180.0 / Math.PI);

            double vx = twist.GetProperty("linear").GetProperty("x").GetDouble();

            Dispatcher.InvokeAsync(() =>
            {
                // Hiển thị lên bảng thông số
                TxtPosX.Text = x.ToString("F3");
                TxtPosY.Text = y.ToString("F3");
                TxtYaw.Text = $"{yawDeg:F1}°";
                TxtVelLinear.Text = vx.ToString("F2");

                // Quy đổi tọa độ mét (x, y) của ROS sang tọa độ Pixel trên WorldCanvas
                double res = _mapResolution > 0 ? _mapResolution : 0.05;
                double canvasX = WORLD_CENTER + (x / res);
                double canvasY = WORLD_CENTER - (y / res); // Trục Y của Canvas ngược chiều với trục Y của ROS

                Canvas.SetLeft(RobotMarker, canvasX);
                Canvas.SetTop(RobotMarker, canvasY);

                // Góc quay trong WPF quay theo chiều kim đồng hồ, trong khi ROS ngược chiều kim đồng hồ -> Đổi dấu (-yawDeg)
                RobotRotation.Angle = -yawDeg;

                //Vẽ thêm điểm vào vệt quỹ đạo di chuyển (nếu xe di chuyển > 3 pixel)
                //var currentPt = new Point(canvasX, canvasY);
                //if (TrajectoryLine.Points.Count == 0 ||
                //    (currentPt - TrajectoryLine.Points[^1]).Length > 2.0)
                //{
                //    TrajectoryLine.Points.Add(currentPt);
                //}
            });
        }

        // =========================================================================
        // 5. ĐIỀU KHIỂN DI CHUYỂN ROBOT (TELEOP NÚT NHẤN & BÀN PHÍM W, A, S, D)
        // =========================================================================
        private async Task PublishCmdVelAsync(double linearX, double angularZ)
        {
            await SendJsonAsync(new
            {
                op = "publish",
                topic = CMD_VEL_TOPIC,
                msg = new
                {
                    header = new { frame_id = "base_link" },
                    twist = new // Thêm trường twist bọc bên ngoài linear/angular
                    {
                        linear = new { x = linearX, y = 0.0, z = 0.0 },
                        angular = new { x = 0.0, y = 0.0, z = angularZ }
                    }
                }
            });
        }

        private void StartMove(double linScale, double angScale)
        {
            _cmdLinearX = linScale * SliderLinear.Value;
            _cmdAngularZ = angScale * SliderAngular.Value;
            _isSendingCmd = true;
            _ = PublishCmdVelAsync(_cmdLinearX, _cmdAngularZ);
        }

        private void StopMove()
        {
            _isSendingCmd = false;
            _cmdLinearX = 0.0;
            _cmdAngularZ = 0.0;
            _ = PublishCmdVelAsync(0.0, 0.0);
        }

        private void BtnForward_Down(object sender, MouseButtonEventArgs e) => StartMove(1.0, 0.0);
        private void BtnBackward_Down(object sender, MouseButtonEventArgs e) => StartMove(-1.0, 0.0);
        private void BtnLeft_Down(object sender, MouseButtonEventArgs e) => StartMove(0.0, 1.0);
        private void BtnRight_Down(object sender, MouseButtonEventArgs e) => StartMove(0.0, -1.0);
        private void BtnMove_Up(object sender, MouseButtonEventArgs e) => StopMove();
        private void BtnStop_Click(object sender, RoutedEventArgs e) => StopMove();

        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            if (TxtRosUrl.IsFocused) return;
            switch (e.Key)
            {
                case Key.W: StartMove(1.0, 0.0); break;
                case Key.S: StartMove(-1.0, 0.0); break;
                case Key.A: StartMove(0.0, 1.0); break;
                case Key.D: StartMove(0.0, -1.0); break;
                case Key.Space: StopMove(); break;
            }
        }

        private void Window_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.W || e.Key == Key.S || e.Key == Key.A || e.Key == Key.D)
            {
                StopMove();
            }
        }

        // =========================================================================
        // 6. TƯƠNG TÁC ZOOM & PAN TRÊN BẢN ĐỒ SLAM GIỐNG RVIZ2
        // =========================================================================
        private void MapViewport_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            double zoomFactor = e.Delta > 0 ? 1.15 : 0.85;
            double newScale = Math.Clamp(MapScale.ScaleX * zoomFactor, 0.4, 15.0);
            MapScale.ScaleX = newScale;
            MapScale.ScaleY = newScale;
        }

        private void MapViewport_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            _isPanning = true;
            _lastMousePos = e.GetPosition(MapViewport);
            MapViewport.CaptureMouse();
        }

        private void MapViewport_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            _isPanning = false;
            MapViewport.ReleaseMouseCapture();
        }

        private void MapViewport_MouseMove(object sender, MouseEventArgs e)
        {
            if (!_isPanning) return;
            Point currentPos = e.GetPosition(MapViewport);
            Vector delta = currentPos - _lastMousePos;
            MapTranslate.X += delta.X;
            MapTranslate.Y += delta.Y;
            _lastMousePos = currentPos;
        }

        private void BtnResetView_Click(object sender, RoutedEventArgs e)
        {
            MapScale.ScaleX = 2.5;
            MapScale.ScaleY = 2.5;
            MapTranslate.X = 0;
            MapTranslate.Y = 0;
        }

        private void BtnClearTrail_Click(object sender, RoutedEventArgs e)
        {
            TrajectoryLine.Points.Clear();
        }
    }
}