/*
=====================================================================
   SUMO ROBOT - PHAT HIEN VIEN SAN (TRANG) + TAN CONG DICH (SIEU AM)
=====================================================================
Phan cung: (giu nguyen so do chan da xac nhan)
  - Arduino UNO
  - Driver dong co L298N (board "NhatMinhESC")
  - Cam bien sieu am HY-SRF05
  - Module do line 5 kenh -> dung lam CAM BIEN BIEN (edge sensor)

SO DO CHAN:
---------------------------------------------------------------------
  L298N ENA -> D5   (PWM toc do Motor 1)
  L298N INA -> D8   (huong Motor 1)
  L298N INB -> D9   (huong Motor 1)
  L298N ENB -> D6   (PWM toc do Motor 2)
  L298N INC -> D10  (huong Motor 2)
  L298N IND -> D11  (huong Motor 2)

  HY-SRF05 TRIG -> D12
  HY-SRF05 ECHO -> D13
  HY-SRF05 VCC  -> 5V, GND -> GND

  Module do bien (trai -> phai):
    OUT1 (trai nhat) -> A4
    OUT2             -> A3
    OUT3 (giua)      -> A2
    OUT4             -> A1
    OUT5 (phai nhat) -> A0

LOGIC SUMO:
---------------------------------------------------------------------
  1) UU TIEN SO 1 - TRANH VIEN TRANG:
     San sumo mau DEN, vien ngoai mau TRANG. Neu bat ky cam bien nao
     thay TRANG (LINE_DETECTED) -> LAP TUC lui + quay dau ve phia DEN
     (vao trong san), bo qua moi hanh dong khac. Uu tien nay CAO HON
     ca viec tan cong, vi rot san la thua ngay.

  2) UU TIEN SO 2 - TAN CONG:
     Neu khong dinh vien, cam bien sieu am thay vat can (doi thu)
     trong pham vi ATTACK_DISTANCE_CM (mac dinh 25cm) -> lao het toc
     luc de day doi thu ra khoi san.

  3) MAC DINH - TIM DICH:
     Neu khong dinh vien va khong thay dich -> xoay tai cho (hoac
     tien cham + xoay) de quet tim doi thu.

LUU Y CAN HIEU CHINH SAU KHI NAP CODE:
---------------------------------------------------------------------
  1) LINE_DETECTED: chua chac chan module bien xuat HIGH hay LOW khi
     gap mau TRANG. Mac dinh dat la HIGH. Neu nap code xong robot
     phan ung SAI (vd: dang o giua san den ma robot cu lui nhu dang
     thay vien trang), doi "#define LINE_DETECTED HIGH" thanh LOW.
  2) Neu mot ben banh xe quay NGUOC chieu mong muon, dao 2 day dong
     co tren domino OUTA/OUTB (hoac OUTC/OUTD) - khong can sua code.
  3) ATTACK_DISTANCE_CM = 25 theo yeu cau. Co the giam xuong (vd 15)
     neu robot lao vao nhau tu xa qua som gay mat on dinh.
  4) BASE_SPEED, ATTACK_SPEED, EDGE_* delay chi la gia tri khoi diem,
     can chinh thuc te theo san dau, do bam banh xe, dien ap pin.
  5) TANG LUC DONG CO TT: PWM (ATTACK_SPEED) da o muc toi da 255 nen
     khong tang duoc bang code nua. Muon motor TT vang manh hon, phai
     tang DIEN AP dau vao L298N: dau 2 pin 3.7V trong khay THEO KIEU
     NOI TIEP (+ pin nay noi - pin kia) de duoc ~7.4V thay vi dau
     song song chi ra 3.7V. Do lai dien ap dau ra khay pin bang VOM
     truoc khi cam vao L298N de chac chan la 7.4V, tranh dau nham
     song song lam robot yeu hoac dau nguoc cuc lam chay IC/pin.
=====================================================================
*/

// ================== CAU HINH CHAN ==================
// --- Driver dong co L298N (NhatMinhESC) ---
#define ENA 5     // PWM toc do Motor 1
#define INA 8     // Huong Motor 1 - chan A
#define INB 9     // Huong Motor 1 - chan B
#define ENB 6     // PWM toc do Motor 2
#define INC 10    // Huong Motor 2 - chan C
#define IND 11    // Huong Motor 2 - chan D

// --- Cam bien sieu am HY-SRF05 ---
#define TRIG_PIN 12
#define ECHO_PIN 13

// --- Module do bien 5 kenh (chi so 0 = trai nhat, 4 = phai nhat) ---
const uint8_t EDGE_PINS[5] = { A4, A3, A2, A1, A0 };

// ================== CAU HINH LOGIC ==================
#define LINE_DETECTED HIGH        // TODO: doi thanh LOW neu test nguoc

#define ATTACK_DISTANCE_CM 5     // Trong 25cm -> lao vao tan cong
#define BASE_SPEED   255          // Toc do di chuyen / lui ne vien (0-255)
#define ATTACK_SPEED 255          // Toc do khi lao vao doi thu (0-255)
#define SEARCH_TURN_SPEED 130     // Toc do xoay khi tim dich (0-255, max)

#define EDGE_BACK_MS  250         // KHONG CON DUNG - da bo buoc lui thang,
                                   // chi con xoay tai cho. Giu lai neu sau
                                   // muon dung lai buoc lui thang.
#define EDGE_TURN_MS  400         // Thoi gian xoay huong vao san sau khi lui

// ================== BIEN TOAN CUC ==================
int lastEdgeSide = 0;   // -1 = vien lech trai, 1 = vien lech phai, 0 = chua co

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  pinMode(ENA, OUTPUT); pinMode(INA, OUTPUT); pinMode(INB, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(INC, OUTPUT); pinMode(IND, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  for (uint8_t i = 0; i < 5; i++) {
    pinMode(EDGE_PINS[i], INPUT);
  }

  stopMotors();
  Serial.begin(9600); // Dung Serial Monitor de debug neu can

  // ---- CHO 5 GIAY SAU KHI BAT CONG TAC (dung luat thi dau sumo) ----
  // Trong luc cho, motor van dung yen (da goi stopMotors() o tren).
  Serial.println("Robot san sang. Cho 5 giay truoc khi bat dau...");
  delay(5000);
  Serial.println("BAT DAU!");
}

// =====================================================================
// VONG LAP CHINH
// =====================================================================
void loop() {
  bool edgeState[5];
  readEdgeSensors(edgeState);

  // ---- UU TIEN 1: DINH VIEN TRANG -> THOAT HIEM NGAY ----
  if (isOnEdge(edgeState)) {
    handleEdge(edgeState);
    return; // bo qua tan cong/tim dich trong chu ky nay
  }

  // ---- UU TIEN 2: PHAT HIEN DICH TRONG TAM DANH ----
  long distance = readDistanceCM();
  if (distance <= ATTACK_DISTANCE_CM) {
    attack();
  } else {
    searchEnemy();
  }
}

// =====================================================================
// DIEU KHIEN DONG CO (muc thap nhat)
// =====================================================================
// speed: -255..255. Duong = tien, am = lui.
void setMotorA(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
    speed = -speed;
  }
  analogWrite(ENA, speed);
}

void setMotorB(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(INC, HIGH);
    digitalWrite(IND, LOW);
  } else {
    digitalWrite(INC, LOW);
    digitalWrite(IND, HIGH);
    speed = -speed;
  }
  analogWrite(ENB, speed);
}

void stopMotors() {
  setMotorA(0);
  setMotorB(0);
}

// =====================================================================
// CAM BIEN SIEU AM HY-SRF05
// =====================================================================
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Timeout 30ms (~5m) de tranh treo chuong trinh neu khong doc duoc
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    return 999; // Khong do duoc -> coi nhu chua thay doi thu
  }
  long distanceCm = duration * 0.0343 / 2;
  return distanceCm;
}

// =====================================================================
// MODULE DO BIEN (EDGE) 5 KENH
// =====================================================================
// true = dang thay mau TRANG (vien san) tai vi tri do
void readEdgeSensors(bool state[5]) {
  for (uint8_t i = 0; i < 5; i++) {
    state[i] = (digitalRead(EDGE_PINS[i]) == LINE_DETECTED);
  }
}

bool isOnEdge(bool state[5]) {
  for (uint8_t i = 0; i < 5; i++) {
    if (state[i]) return true;
  }
  return false;
}

// =====================================================================
// XU LY KHI DINH VIEN TRANG: LUI + QUAY DAU VE PHIA DEN (TRONG SAN)
// =====================================================================
void handleEdge(bool state[5]) {
  // Xac dinh vien nam ben trai hay ben phai de biet huong quay vao trong
  bool edgeLeft  = state[0] || state[1];   // 2 cam bien trai nhat
  bool edgeRight = state[3] || state[4];   // 2 cam bien phai nhat
  bool edgeCenter = state[2];              // cam bien giua

  int side;
  if (edgeRight && !edgeLeft) {
    side = 1;   // vien o ben phai -> can quay sang trai de vao trong
  } else if (edgeLeft && !edgeRight) {
    side = -1;  // vien o ben trai -> can quay sang phai de vao trong
  } else if (edgeCenter) {
    // Dinh chinh dien (ca 2 ben hoac chi giua) -> dung lai lan quay truoc
    side = (lastEdgeSide != 0) ? lastEdgeSide : 1;
  } else {
    side = (lastEdgeSide != 0) ? lastEdgeSide : 1;
  }
  lastEdgeSide = side;

  // XOAY TAI CHO NGAY (2 banh quay NGUOC CHIEU nhau) - khong lui thang
  // truoc nua, vi lui thang 2 banh cung chieu de bi lech/1 banh khong
  // an luc gay truot/vang khoi san. Xoay tai cho an toan hon.
  if (side == 1) {
    // quay sang trai
    setMotorA(-SEARCH_TURN_SPEED);
    setMotorB(SEARCH_TURN_SPEED);
  } else {
    // quay sang phai
    setMotorA(SEARCH_TURN_SPEED);
    setMotorB(-SEARCH_TURN_SPEED);
  }
  delay(EDGE_TURN_MS);
  stopMotors();
  delay(50);

  // Sau buoc nay, vong loop() se doc lai cam bien bien de kiem tra
  // da an toan chua; neu van con dinh vien no se lap lai quy trinh nay.
}

// =====================================================================
// TAN CONG: LAO THANG VAO DOI THU HET TOC LUC
// =====================================================================
void attack() {
  setMotorA(ATTACK_SPEED);
  setMotorB(ATTACK_SPEED);
}

// =====================================================================
// TIM DICH: XOAY QUET KHI CHUA THAY DOI THU
// =====================================================================
void searchEnemy() {
  // Xoay tai cho de quet radar sieu am quanh minh.
  // Neu muon vua tien vua xoay (quet rong hon), doi 2 dong duoi thanh:
  //   setMotorA(BASE_SPEED / 2);
  //   setMotorB(BASE_SPEED);
  setMotorA(-SEARCH_TURN_SPEED);
  setMotorB(SEARCH_TURN_SPEED);
}
