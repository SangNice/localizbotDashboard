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

        private const string CMD_VEL_TOPIC = "/diff_cont/cmd_vel";
        private const string ODOM_TOPIC = "/odometry/filtered";

        // Thông số bản đồ SLAM hiện tại để quy đổi tọa độ (m) -> Pixel 
        private double _mapResolution = 0.05;
        private int _mapHeight = 0;
        private const double WORLD_CENTER = 1000.0;

        // =========================================================================
        // BIẾN LƯU TRỮ HỆ TỌA ĐỘ SLAM (KẾT HỢP MAP -> ODOM VÀ ODOM -> BASE_FOOTPRINT)
        // =========================================================================
        // 1. Tọa độ tương đối từ EKF (odom -> base_footprint): Có thể bị trôi theo thời gian do trượt bánh
        private double _odomX = 0.0;
        private double _odomY = 0.0;
        private double _odomYawRad = 0.0;
        private double _linearVelX = 0.0;

        // 2. Độ lệch hiệu chỉnh từ SLAM Toolbox (map -> odom): Giúp kéo xe về đúng vị trí thật trên Bản đồ
        private double _mapToOdomX = 0.0;
        private double _mapToOdomY = 0.0;
        private double _mapToOdomYawRad = 0.0;

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
                await SubscribeTopicAsync("/map", "nav_msgs/msg/OccupancyGrid", 4000);
                await SubscribeTopicAsync(ODOM_TOPIC, "nav_msgs/msg/Odometry", 50);

                // QUAN TRỌNG CHO BÀI TOÁN SLAM: Đăng ký nhận cây tọa độ /tf để lấy phép biến đổi (map -> odom)
                // Đây chính là dữ liệu mà SLAM dùng bản đồ để định vị ngược lại chiếc xe!
                await SubscribeTopicAsync("/tf", "tf2_msgs/msg/TFMessage", 50);

                await SubscribeTopicAsync("/imu/data", "sensor_msgs/msg/Imu", 100);
                await SubscribeTopicAsync("/ultrasonic/range", "sensor_msgs/msg/Range", 100);

                // Quảng bá topic điều khiển vận tốc
                await SendJsonAsync(new
                {
                    op = "advertise",
                    topic = CMD_VEL_TOPIC,
                    type = "geometry_msgs/msg/TwistStamped"
                });

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
        // 2. VÒNG LẶP NHẬN & GIẢI MÃ DỮ LIỆU TỪ ROS 2
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

                if (!root.TryGetProperty("op", out JsonElement opElem)) return;
                string op = opElem.GetString() ?? "";

                // Nếu nhận được phản hồi từ các lệnh Lưu/Tải/Reset Map (Service Response)
                if (op == "service_response")
                {
                    string serviceName = root.GetProperty("service").GetString() ?? "";
                    Dispatcher.InvokeAsync(() =>
                    {
                        TxtMapInfo.Text = $"✔ Đã thực thi xong dịch vụ: {serviceName}";
                    });
                    return;
                }

                if (op != "publish") return;

                string topic = root.GetProperty("topic").GetString() ?? "";
                JsonElement msg = root.GetProperty("msg");

                if (topic == "/map")
                {
                    HandleMapMessage(msg);
                }
                else if (topic == "/tf")
                {
                    HandleTfMessage(msg); // Giải mã ma trận định vị SLAM (map -> odom)
                }
                else if (topic == ODOM_TOPIC)
                {
                    HandleOdomMessage(msg); // Giải mã tọa độ EKF (odom -> base_footprint)
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
                            ? new SolidColorBrush(Color.FromRgb(239, 68, 68))
                            : new SolidColorBrush(Color.FromRgb(251, 146, 60));
                    });
                }
            }
            catch { }
        }

        // =========================================================================
        // 3. DỰNG BẢN ĐỒ SLAM GIỐNG RVIZ2
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

            byte[] pixels = new byte[width * height * 4];
            int idx = 0;

            foreach (JsonElement cell in dataArray.EnumerateArray())
            {
                int val = cell.GetInt32();
                int mapX = idx % width;
                int mapY = idx / width;

                int flippedY = (height - 1) - mapY;
                int pixelIndex = (flippedY * width + mapX) * 4;

                byte r, g, b;
                if (val == -1) { r = 39; g = 39; b = 42; }
                else if (val == 0) { r = 212; g = 212; b = 216; }
                else if (val >= 65) { r = 9; g = 9; b = 11; }
                else
                {
                    byte shade = (byte)(210 - (val * 2));
                    r = shade; g = shade; b = shade;
                }

                pixels[pixelIndex + 0] = b;
                pixels[pixelIndex + 1] = g;
                pixels[pixelIndex + 2] = r;
                pixels[pixelIndex + 3] = 255;
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

                double leftPx = WORLD_CENTER + (originX / resolution);
                double topPx = WORLD_CENTER - (originY / resolution) - height;

                Canvas.SetLeft(SlamMapImage, leftPx);
                Canvas.SetTop(SlamMapImage, topPx);

                TxtMapInfo.Text = $"Kích thước: {width}x{height} px | Độ phân giải: {resolution:F3} m/px";
            });
        }

        // =========================================================================
        // 4. THUẬT TOÁN ĐỊNH VỊ XE TRÊN BẢN ĐỒ (KẾT HỢP TF MAP->ODOM VÀ ODOM->BASE)
        // =========================================================================
        // Hàm 4.1: Đọc phép biến đổi sửa sai số (map -> odom) do slam_toolbox phát ra khi so khớp Lidar với Map
        private void HandleTfMessage(JsonElement msg)
        {
            if (!msg.TryGetProperty("transforms", out JsonElement transforms)) return;

            foreach (JsonElement tf in transforms.EnumerateArray())
            {
                string parentFrame = tf.GetProperty("header").GetProperty("frame_id").GetString()?.TrimStart('/') ?? "";
                string childFrame = tf.GetProperty("child_frame_id").GetString()?.TrimStart('/') ?? "";

                // Bắt lấy đúng mắt xích (map -> odom) mà SLAM Toolbox tính toán ra
                if (parentFrame == "map" && childFrame == "odom")
                {
                    JsonElement trans = tf.GetProperty("transform").GetProperty("translation");
                    JsonElement rot = tf.GetProperty("transform").GetProperty("rotation");

                    _mapToOdomX = trans.GetProperty("x").GetDouble();
                    _mapToOdomY = trans.GetProperty("y").GetDouble();

                    double qx = rot.GetProperty("x").GetDouble();
                    double qy = rot.GetProperty("y").GetDouble();
                    double qz = rot.GetProperty("z").GetDouble();
                    double qw = rot.GetProperty("w").GetDouble();

                    double siny_cosp = 2.0 * (qw * qz + qx * qy);
                    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                    _mapToOdomYawRad = Math.Atan2(siny_cosp, cosy_cosp);

                    UpdateRobotMapPose();
                }
            }
        }

        // Hàm 4.2: Đọc vị trí tương đối từ bộ lọc EKF (odom -> base_footprint)
        private void HandleOdomMessage(JsonElement msg)
        {
            JsonElement pose = msg.GetProperty("pose").GetProperty("pose");
            JsonElement pos = pose.GetProperty("position");
            JsonElement ori = pose.GetProperty("orientation");
            JsonElement twist = msg.GetProperty("twist").GetProperty("twist");

            _odomX = pos.GetProperty("x").GetDouble();
            _odomY = pos.GetProperty("y").GetDouble();

            double qx = ori.GetProperty("x").GetDouble();
            double qy = ori.GetProperty("y").GetDouble();
            double qz = ori.GetProperty("z").GetDouble();
            double qw = ori.GetProperty("w").GetDouble();

            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            _odomYawRad = Math.Atan2(siny_cosp, cosy_cosp);

            _linearVelX = twist.GetProperty("linear").GetProperty("x").GetDouble();

            UpdateRobotMapPose();
        }

        // Hàm 4.3: Nhân ma trận biến đổi 2D: T_map_base = T_map_odom * T_odom_base
        // Đây là cốt lõi của việc dùng Bản đồ đã quét để xác định chính xác vị trí xe!
        private void UpdateRobotMapPose()
        {
            // Công thức biến đổi hệ tọa độ phẳng 2D (Rigid Body Transformation):
            double cosTheta = Math.Cos(_mapToOdomYawRad);
            double sinTheta = Math.Sin(_mapToOdomYawRad);

            double mapX = _mapToOdomX + (_odomX * cosTheta - _odomY * sinTheta);
            double mapY = _mapToOdomY + (_odomX * sinTheta + _odomY * cosTheta);
            double mapYawRad = _mapToOdomYawRad + _odomYawRad;

            // Chuẩn hóa góc về khoảng [-PI, PI]
            mapYawRad = Math.Atan2(Math.Sin(mapYawRad), Math.Cos(mapYawRad));
            double mapYawDeg = mapYawRad * (180.0 / Math.PI);
            double driftYawDeg = _mapToOdomYawRad * (180.0 / Math.PI);

            Dispatcher.InvokeAsync(() =>
            {
                // Hiển thị tọa độ chuẩn của xe trên hệ quy chiếu Bản đồ (Map Frame)
                TxtPosX.Text = mapX.ToString("F3");
                TxtPosY.Text = mapY.ToString("F3");
                TxtYaw.Text = $"{mapYawDeg:F1}°";
                TxtVelLinear.Text = _linearVelX.ToString("F2");

                // Hiển thị độ lệch (Drift) mà SLAM đã phát hiện và bù trừ cho bánh xe
                TxtTfCorrection.Text = $"Sai số Odom được SLAM bù: ΔX={_mapToOdomX:F2}m, ΔY={_mapToOdomY:F2}m, ΔYaw={driftYawDeg:F1}°";

                // Quy đổi tọa độ Map (x, y) sang Pixel trên WorldCanvas
                double res = _mapResolution > 0 ? _mapResolution : 0.05;
                double canvasX = WORLD_CENTER + (mapX / res);
                double canvasY = WORLD_CENTER - (mapY / res);

                Canvas.SetLeft(RobotMarker, canvasX);
                Canvas.SetTop(RobotMarker, canvasY);
                RobotRotation.Angle = -mapYawDeg;

                // Vẽ quỹ đạo giới hạn tối đa 250 điểm để vừa đẹp vừa không bao giờ bị lag giao diện
                var currentPt = new Point(canvasX, canvasY);
                if (TrajectoryLine.Points.Count == 0 ||
                    (currentPt - TrajectoryLine.Points[^1]).Length > 3.0)
                {
                    if (TrajectoryLine.Points.Count > 250)
                        TrajectoryLine.Points.RemoveAt(0);
                    TrajectoryLine.Points.Add(currentPt);
                }
            });
        }

        // =========================================================================
        // 5. CÁC DỊCH VỤ QUẢN LÝ BẢN ĐỒ SLAM (LƯU MAP, TẢI MAP ĐỊNH VỊ, RESET MAP)
        // =========================================================================
        // 5.1: Lưu bản đồ vừa quét xong xuống bộ nhớ Raspberry Pi (.posegraph & .data)
        private async void BtnSaveMap_Click(object sender, RoutedEventArgs e)
        {
            string filename = TxtMapPath.Text.Trim();
            if (string.IsNullOrEmpty(filename)) return;

            await SendJsonAsync(new
            {
                op = "call_service",
                service = "/slam_toolbox/serialize_map",
                args = new { filename = filename }
            });
            TxtMapInfo.Text = $"Đang lưu bản đồ vào: {filename}...";
        }

        // 5.2: Tải bản đồ đã lưu lên để định vị xe (match_type = 1: Khởi tạo tại gốc hoặc khớp vị trí)
        private async void BtnLoadMap_Click(object sender, RoutedEventArgs e)
        {
            string filename = TxtMapPath.Text.Trim();
            if (string.IsNullOrEmpty(filename)) return;

            await SendJsonAsync(new
            {
                op = "call_service",
                service = "/slam_toolbox/deserialize_map",
                args = new
                {
                    filename = filename,
                    match_type = 1, // 1 = START_AT_FIRST_NODE (Định vị xe bắt đầu từ gốc Map đã lưu)
                    initial_pose = new { x = 0.0, y = 0.0, theta = 0.0 }
                }
            });
            TxtSlamModeTitle.Text = "SLAM VIEW (ĐANG ĐỊNH VỊ TRÊN MAP ĐÃ LƯU)";
            TxtMapInfo.Text = $"Đang tải bản đồ định vị từ: {filename}...";
        }

        // 5.3: Xóa bản đồ hiện tại để quét lại từ đầu
        private async void BtnResetSlam_Click(object sender, RoutedEventArgs e)
        {
            await SendJsonAsync(new
            {
                op = "call_service",
                service = "/slam_toolbox/reset",
                args = new { }
            });
            TrajectoryLine.Points.Clear();
            TxtSlamModeTitle.Text = "SLAM VIEW (MAPPING & LOCALIZATION)";
            TxtMapInfo.Text = "Đã gửi lệnh Reset bản đồ SLAM!";
        }

        // =========================================================================
        // 6. ĐIỀU KHIỂN DI CHUYỂN ROBOT (TELEOP NÚT NHẤN & BÀN PHÍM W, A, S, D)
        // =========================================================================
        private async Task PublishCmdVelAsync(double linearX, double angularZ)
        {
            await SendJsonAsync(new
            {
                op = "publish",
                topic = CMD_VEL_TOPIC,
                msg = new
                {
                    header = new { frame_id = "base_footprint" },
                    twist = new
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
            if (TxtRosUrl.IsFocused || TxtMapPath.IsFocused) return;
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
        // 7. TƯƠNG TÁC ZOOM & PAN TRÊN BẢN ĐỒ SLAM
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