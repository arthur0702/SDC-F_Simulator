# 士林電機 SDC-F 多軸伺服驅動器模擬器 (CC-Link IE Field Basic + SLMP)

一套使用純 Python（`PySide6` + `qasync` + `asyncio`）開發的 **士林電機（Shihlin Electric）SDC-F 系列伺服驅動器多軸模擬器**。  
本工具可直接在 Windows PC 上模擬 1 至 8 軸支援 **CC-Link IE Field Basic (IEFB)** 與 **SLMP** 通訊協定的伺服驅動器，與三菱電機 **FX5U PLC** 及 **GX Works3** 進行真實乙太網路循環通訊與運動控制功能塊（FB）驗證，無須接實體馬達即可測試 PLC 順控與運動邏輯。

---

## 目錄

- [核心特色](#核心特色)
- [通訊協定與支援清單](#通訊協定與支援清單)
- [檔案架構](#檔案架構)
- [系統需求與安裝](#系統需求與安裝)
- [使用說明](#使用說明)
- [開發與問題排除報告](#開發與問題排除報告)
- [MIT 授權與免責聲明](#mit-授權與免責聲明)
- [AI 協助開發聲明](#ai-協助開發聲明)
- [資料來源](#資料來源)

---

## 核心特色

1. **真實對接三菱 FX5U PLC 與 GX Works3**
   - 支援 **SLMP 3E/4E Binary (`0E70h` CyclicData)** 循環通訊，完整實作經過實機 SDC-F 驗證的 **131 Bytes `wfirst` 回應格式**（`11B SLMP + 48B IEFB + 64B RWr + 8B RX`）。
   - 支援 **NodeSearch (`0E30h`) / IPAddressSet (`0E31h`)** 站台搜尋與 **iQSS (`45237`)** 裝置探索，可直接出現在 GX Works3「CC-Link IEF Basic 網路組態」自動偵測清單中。
   - 同時支援 **UDP 5010 與 TCP 5010** 之 **SLMP `4020h`** 物件字典讀寫（單筆讀寫 `0001h`/`0002h` 與 SubID 區塊讀寫 `0005h`/`0006h`）。

2. **完整 CiA 402 狀態機與物理運動模擬引擎**
   - 支援 **PP (Profile Position, Mode 1)**、**PV (Profile Velocity, Mode 3)**、**PT (Profile Torque, Mode 4)**、**HM (Homing, Mode 6)** 四種運轉模式。
   - 支援 24-bit 編碼器解析度 (`608Fh:01`，預設 `16,777,216 pulse/rev` 與可選 22-bit / 17-bit / 10000)、電子齒輪比 (`2006h`/`2007h`)、`PA53` 三種 PDO 映射模式（Mode 1 / Mode 2 / Mode 3 自由映射）、`PA54` 速度加減速單位切換（`0.01 r/min & ms` 或 `pps & pps²`）。
   - 即時維護狀態字組 `6041h` 與士林/三菱延伸狀態字組 `2D11h`（`S_SA`/`S_INP`/`S_TLC`）與 `2D12h`（`S_ZSP`/`S_VLC`）。

3. **多軸動態視覺化與互動模擬面板**
   - 即時顯示各軸馬達轉子旋轉動畫、線性滑台、CSR/SSR LED、`CW/SW`、`6064h` 位置、`606Ch` 速度、`60F4h` 追隨誤差。
   - 支援單軸手動觸發虛擬感測器（`LSN`、`ORGP`、`LSP`）與常見異警注入／復歸（`AL.03 過電流`、`AL.08 誤差過大`、`AL.01 過電壓`、`AL.05 過負載`）。
   - 內建 **Windows 多靜態 IP 綁定與防火牆放行批次腳本 (`.bat`)** 產生器與 **SLMP `4020h` 測試發送器分頁**。

---

## 通訊協定與支援清單

### 網路通訊埠

| 通訊埠 | 協定 | 功能說明 |
| :--- | :--- | :--- |
| **UDP 61450** | CC-Link IEFB / SLMP `0E70h` | PLC 與伺服之間的週期性循環資料交換（`RY`/`RWw` ↔ `RX`/`RWr`） |
| **UDP 61451** | SLMP `0E30h` / `0E31h` | GX Works3 站台探索（NodeSearch）與線上 IP 設定 |
| **UDP / TCP 5010** | SLMP `4020h` | 物件字典非週期性參數讀寫（`0001h`/`0002h`/`0005h`/`0006h`） |
| **UDP 45237** | iQSS Discovery | 三菱 iQ Sensor Solution 網路搜尋與識別回應 |

### 已實機驗證支援之運動控制功能塊（FX5U + GX Works3）

#### 三菱官方運動控制 FB（`MC_*_CCLinkIEFBasic_F`）
- `MC_Power_CCLinkIEFBasic_F`（Servo ON / OFF，`RY3F`/`RX3F`/`RX3E` 與 CiA 402 PDS 切換）
- `MC_Home_CCLinkIEFBasic_F`（原點復歸，含實機各型 `6098h` LSN / LSP / ORGP 感測器復歸模式）
- `MC_MoveRelative_CCLinkIEFBasic_F` / `MC_MoveAbsolute_CCLinkIEFBasic_F`（PP 定位控制，含 `6040h` Bit 4 New Set-point 觸發與 `2D11h` S_INP 回授，已於單軸／多站含保留站架構下實測通過）
- `MC_MoveVelocity_CCLinkIEFBasic_F`（PV 速度控制與速度平滑加減速，支援 `LSP`/`LSN` 極限減速停機，無 Error 4611）
- `MC_Stop_CCLinkIEFBasic_F` / `MC_Reset_CCLinkIEFBasic_F`（停止與異警復歸）

#### 非官方／自訂運動控制 FB
- `MC_Jog`（**非三菱官方 FB**，為自訂／第三方 JOG 寸動正反轉功能塊；同樣會檢查 `RX3E` 就緒與 `6060h = 3`，已實機驗證動作正常、無 Error 4611）

---

## 檔案架構

```text
.
├── main_gui.py                        # PySide6 + qasync 圖形化主視窗與多軸視覺化面板
├── iefb_cyclic_server.py              # UDP 61450/61451/45237 伺服器、多軸廣播分派與 netsh 腳本產生
├── slmp_server.py                     # UDP/TCP 5010 SLMP 3E/4E 封包解析與 4020h 測試客戶端
├── servo_axis.py                      # CiA 402 狀態機、PP/PV/PT/HM 物理引擎與極限/異警模擬
├── object_dictionary.py               # SDC-F 物件字典 (1000h~6FFFh) 與 Mode 1/2/3 PDO 映射
├── README.md                          # 專案使用說明、授權與聲明文件
├── LICENSE                            # MIT 授權條款全文
└── DEV_AND_TROUBLESHOOTING_REPORT.md  # 完整開發架構與實機對接問題排除報告
```

---

## 系統需求與安裝

- **作業系統**：Windows 10 / 11（建議連接實體乙太網路卡與 PLC 同一網段）
- **Python 版本**：Python 3.10 以上

安裝相依套件：

```bash
pip install PySide6 qasync
```

啟動模擬器：

```bash
python main_gui.py
```

---

## 使用說明

### 1. 設定 Windows 多 IP 與防火牆（對接實體 FX5U 時必做）

1. 將電腦有線乙太網卡接上與 FX5U PLC 相連的交換器或直連網線（預設 PLC 主站常見為 `192.168.3.250`，子網路遮罩 `255.255.255.0`）。
2. 在模擬器上方設定 **軸數**（例如 `1` 或 `2`）與 **起始 IP**（例如 `192.168.3.1` 或避開路由器閘道的 `192.168.3.11`）。
3. 點擊 **「一鍵產生 Windows 多 IP 與防火牆腳本」**，輸入您的 Windows 網卡名稱（預設 `乙太網路`，英文系統請改為 `Ethernet`），另存或複製為 `.bat` 檔後，**以系統管理員身分執行**。
4. 回到模擬器點擊 **「啟動全部通訊埠」**（程式啟動時亦會自動嘗試啟動）。當軸卡片右上角顯示 `UDP: ON` 且 `CSR(RY)`、`SSR(RX)` 亮綠/藍燈時，代表循環通訊已建立成功。
5. **恢復電腦原廠網路**：測試結束後，可點擊 **「產生恢復原廠網路腳本（.bat）」**，選擇「恢復自動取得 IP (DHCP)」、「僅移除模擬軸 IP」或「重設為單一固定 IP」，並可一併移除模擬器防火牆規則。

> **注意事項**：若區域網路中已有路由器/閘道器佔用 `192.168.3.1`，建議將起始 IP 改為 `192.168.3.11`（並在 GX Works3 對應修改從站 IP），避免 ARP 位址衝突導致 PLC 回報 `CFE8` 無回應。

### 2. GX Works3 網路組態設定

1. 於 GX Works3 開啟 `導覽 -> 參數 -> 模組參數 -> 乙太網路埠 -> 基本設定 -> CC-Link IEF Basic 設定`。
2. 點擊 **「網路組態設定」**，可使用 **「連線機器自動偵測」** 透過 UDP `61451` (`0E30h`) 直接搜尋到模擬的 `SDC-F` 軸，或手動拖曳 `SDC-F` CSP+ 裝置至拓樸並設定相同 IP（佔用站數 1 站，RX/RY 64 點、RWr/RWw 32 字）。
3. 確認套用並將參數寫入 FX5U PLC，重新上電或 RUN 後即可透過三菱標準 FB 控制模擬軸。

### 3. 參數切換與診斷面板操作

- **上方控制列**：
  - **軸數 / 起始 IP**：動態增減 1~8 軸並自動遞增 IP。
  - **PA53 映射**：支援 `0: Mode 1`（預設，網路標準模式 1）、`1: Mode 2`（含 6098h/6099h 原點復歸映射）、`2: Mode 3`（由 `1600h`/`1A00h` 動態自由映射）。
  - **PA54 單位**：支援 `0: 0.01 r/min & ms`（預設）與 `1: pulse/s & pulse/s²`。
  - **馬達解析度**：可切換 24-bit（`16,777,216`）、22-bit、17-bit 或 `10,000 pulse/rev`。
- **左側軸卡片**：可於運轉中點擊 `LSN` / `ORGP` / `LSP` 模擬極限與原點感測器，亦可測試 **異警注入** 與 **異警復歸**。
- **右側分頁面板**：
  1. **PDO 監控 (RWwn / RWrn)**：即時解碼顯示 32-Word 控制字與狀態字，以及 64-bit `RY`/`RX` 原始 Hex。
  2. **SLMP 物件字典 (1000h~6FFFh)**：支援關鍵字篩選與雙擊直接修改參數（且會自動填入測試分頁參數）。
  3. **SLMP 通訊日誌**：記錄連線恢復/逾時、`PDO_CHG` 關鍵控制變更、`4020h`/`0E30h` 封包交易明細。
  4. **SLMP 4020h 測試**：可主動發送真實 SLMP 3E UDP 封包至 Port 5010 測試讀寫任意物件字典。

---

## 開發與問題排除報告

完整的協議逆向工程分析、多軸網路設計以及與 FX5U / 實體 SDC-F 對接過程中解決的所有關鍵問題（包含 `0E70h` 131B `wfirst` 格式對齊、`RX3E` 解決 `Error 4611`、`2D11h`/`2D12h` 狀態位元、`2D20h` 限速邊界、`4020h` 讀取長度對齊與電子齒輪換算），請參閱：

👉 **[DEV_AND_TROUBLESHOOTING_REPORT.md](./DEV_AND_TROUBLESHOOTING_REPORT.md)**

---

## MIT 授權與免責聲明

本專案採用 **[MIT License](./LICENSE)** 授權條款釋出，任何人皆可依條款自由使用、複製、修改、合併、發布及散佈本軟體。

### 免責聲明 (Disclaimer)
1. **非官方模擬軟體**：本專案為獨立開發之通訊協議學習、PLC 離線/桌面測試與開發輔助工具，**並非士林電機（Shihlin Electric）或三菱電機（Mitsubishi Electric）官方產品**，亦與上述公司無任何從屬、贊助或背書關係。文中提及之商標、型號與軟體名稱僅作技術相容性說明之用，其權利均屬原公司所有。
2. **禁止直接用於實際安全關鍵控制**：本模擬器僅供非實機環境之通訊流程驗證與教育測試使用，未經完整實體設備驗證前，請勿直接依據模擬結果作為現場機械安全防護、人身安全或高風險工業設備運作的唯一依據。
3. **無擔保條款**：本軟體依「現況（AS IS）」提供，不附帶任何明示或暗示的擔保（包含但不限於適售性、特定用途適用性或無侵權保證）。作者及貢獻者對於使用或無法使用本軟體所導致的任何直接或間接損害（包含設備損壞、資料遺失或營業中斷）概不負責。

---

## AI 協助開發聲明

本專案在設計架構、通訊協議封包解析與重組、CiA 402 狀態機模擬邏輯、PySide6 圖形化介面實作、繁體中文程式碼註解撰寫，以及對接實體三菱 FX5U PLC 與士林 SDC-F 伺服時的除錯分析過程中，均有採用 **生成式 AI 程式開發助手** 協助撰寫程式碼、整理封包 HEX 比對及產生技術文件。所有通訊相容性與運動功能塊驗證均經過實體 FX5U PLC 與現場抓包比對確認。

---

## 資料來源

本專案實作所參考與依據的公開技術規格與實機比對來源如下：

1. **士林電機 (Shihlin Electric) 公開技術手冊與描述檔**
   - `SDC-F` 系列交流伺服驅動器 CC-Link IE Field Basic 通訊手冊（PA53 / PA54、Mode 1 / Mode 2 / Mode 3 PDO 映射、2000h~2FFFh 廠商擴充物件與 `2D11h`/`2D12h`/`2D20h` 定義）
   - 士林電機 CSP+ 裝置描述檔：`0x0370_SDC-F_E_CCIEFBasic__1_en.xml`（Vendor Code `0x0370`、Product ID `0x000D`、通訊逾時與車站佔用數定義）
2. **CLPA (CC-Link Partner Association) 與三菱電機公開規範**
   - CC-Link IE Field Basic (SLMP `0E70h` CyclicData、`0E30h` NodeSearch、`0E31h` IPAddressSet) 通訊協定結構
   - SLMP (Seamless Message Protocol) 3E/4E Binary Frame 與 Command `4020h` 物件字典存取規範
   - 三菱電機 MELSEC iQ-F `FX5U` 與 CC-Link IEF Basic 專用運動控制功能塊（`MC_*_CCLinkIEFBasic_F`）公開應用手冊及錯誤碼定義（如 Error `4611h` / `CFE8h`）
3. **CiA (CAN in Automation) 公開標準**
   - **CiA 402 (IEC 61800-7)** Drives and Motion Control Device Profile（PDS 狀態機、`6040h`/`6041h` 控制/狀態字、`6060h`/`6061h` 操作模式、PP/PV/PT/HM 物件定義）
4. **實機封包擷取與對照驗證**
   - 由開發與測試團隊於實體三菱 `FX5U` PLC 與實體士林 `SDC-F` 伺服驅動器之間，透過系統日誌與封包探測所實測比對之 `0E70h`（131B `wfirst` 回包）、`RX3E/RX3F`（`0xC0`）及 `4020h` 實際通訊特徵。
