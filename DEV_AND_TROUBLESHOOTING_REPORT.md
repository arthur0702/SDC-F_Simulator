# 士林電機 SDC-F 多軸伺服模擬器 — 開發架構與問題排除工程報告

本報告完整記錄「士林電機 SDC-F 多軸伺服驅動器模擬器（CC-Link IE Field Basic + SLMP）」從系統架構設計、與實體三菱 FX5U PLC 及實體 SDC-F 伺服對接除錯，到最終順利通過三菱官方 FB（`MC_Power`、`MC_Home`、`MC_MoveRelative`、`MC_MoveVelocity_CCLinkIEFBasic_F` 等）與非官方／自訂 `MC_Jog` 功能塊驗證的完整開發與問題排除歷程。

---

## 一、 系統整體架構設計

本模擬器以純 Python 非同步架構為核心，透過 `qasync` 將 Python 原生 `asyncio` 事件迴圈與 `PySide6` GUI 事件迴圈整合於同一執行緒中，達成低延遲（~10ms 物理步進 + 即時 UDP/TCP 回應）與零跨執行緒鎖定風險的架構：

| 模組 | 職責說明 |
| :--- | :--- |
| `object_dictionary.py` | 實作 SDC-F 完整 CiA 402 + 士林廠商參數物件字典 (`1000h`~`6FFFh`)、`PA53` (Mode 1/2/3) 32-Word PDO 編解碼，以及 SLMP `4020h` 權限/長度/範圍驗證。 |
| `servo_axis.py` | 實作單軸伺服引擎：CiA 402 PDS 狀態機、PP/PV/PT/HM 物理運動積分、24-bit 解析度與電子齒輪換算、`2D11h`/`2D12h` 狀態輸出、虛擬感測器與異警注入。 |
| `slmp_server.py` | 實作 **UDP 5010 + TCP 5010** 雙協定 SLMP 3E/4E 伺服器（支援 `4020h` 物件讀寫與 `0E30h`/`0E31h` NodeSearch），並提供內建測試客戶端 `SLMPClientTester`。 |
| `iefb_cyclic_server.py` | 實作 **UDP 61450** (`0E70h` CyclicData)、**UDP 61451** (NodeSearch) 與 **UDP 45237** (iQSS) 非同步伺服器，含多軸獨立靜態 IP 綁定、`0.0.0.0` 廣播補漏、30ms 跨 Socket 去重及看門狗監控。 |
| `main_gui.py` | 工業深色風格主視窗，提供多軸動態卡片網格、PDO 監控表、物件字典瀏覽/修改、通訊日誌、SLMP `4020h` 測試分頁與 Windows `netsh` 腳本產生器。 |

---

## 二、 實機對接與問題排除完整歷程

在與實體 **FX5U PLC (GX Works3)** 進行實機連線測試期間，陸續遭遇並解決了多項文件未明寫或實機與標準有細微差異的關鍵問題。以下依時間軸與技術領域詳述問題現象、根因定位與解決方案：

### 問題 1：FX5U 找不到模擬軸或偶發斷線 (`CFE8`)、廣播封包漏接與 `.1` IP 衝突
- **問題現象**：
  - 多軸情境下模擬軸各自綁定 `192.168.3.1`、`192.168.3.2` 等單播 IP 時，無法收到 FX5U 送往廣播位址的 `0E30h` NodeSearch 或廣播型 `0E70h` 循環封包；若只綁 `0.0.0.0` 又可能由錯誤來源 IP 回覆，造成 PLC ARP 錯亂。
  - 部分現場網段若剛好存在路由器或交換器佔用 `192.168.3.1`，導致 PLC `CFE8` 無回應斷線。
- **根因分析**：
  - 在 Windows 上僅綁定特定單播 IPv4 位址的 UDP Socket 不會接收 subnet/limited broadcast 封包。
  - 若同一個封包同時被單播 Socket 與廣播 Socket 收到，會導致同一週期處理兩次甚至重複計算物理位置與計數。
- **解決方案**：
  - 在 `MultiAxisCyclicManager` 建立 **「特定 IP Socket + `0.0.0.0` 廣播伴隨接聽器」雙層架構**：一律由對應軸的靜態 IP Socket（`reply_transport_fn`）回傳封包，確保來源 IP 與 ARP 正確。
  - 在 `_IEFBCyclicProtocol` 與 `_SharedNodeSearchBroadcastProtocol` 實作 **跨 Socket 去重快取 (`dedup_cache`)**（比對封包標頭前 64B 與來源位址，30ms 內不重複處理）。
  - 於 `diagnose_local_network()` 加入 `.1` 常見閘道器尾碼衝突偵測警告與 OS 出站路由來源 IP 檢查，並提供一鍵產出 Windows `netsh` 多 IP 綁定與防火牆腳本功能。

---

### 問題 2：FX5U 送出 SLMP `0E70h` 但無法確立 CSR/SSR 握手（封包格式差異）
- **問題現象**：
  - 模擬器已收到 FX5U 傳來的 UDP 61450 封包，但 PLC 側遲遲無法完成建立連線，CSR (`RY3F`) 無法持續保持或 `SSR` 未被主站承認。
- **根因分析**：
  - 透過實機封包探測工具直連實體 **士林 SDC-F 伺服** 擷取真實回包比對，確認 FX5U 送出的 `0E70h` 請求封包並非傳統單純 24B 標頭，而是 **143 Bytes 的 SLMP 3E + Ver.2 擴充標頭**，且其 PDO 資料區順序為 **`[RWw 64B][RY 8B]`**（Word 區在前、Bit 區在後）。
  - 相對應地，實體 SDC-F 回傳給 FX5U 的標準回包必須為精確的 **131 Bytes (`slmp11_real_sdcf_48b_wfirst`)**：
    - `0..10` (11B)：SLMP 3E 回應標頭 (`D0 00 00 FF FF 03 00 7A 00 00 00`)
    - `11..58` (48B)：IEFB Ver.2 回應標頭（`Offset=40`、`Vendor=0x0370`、`Product=0x000D`、`OccupiedStations=1`、Little-Endian SlaveIP、GroupNo、FrameSeq）
    - `59..122` (64B)：`RWr` 區（與主站請求對稱，Word 在前）
    - `123..130` (8B)：`RX` 區（Bit 在後，第 130 byte MSB 對應 `RX3F` SSR）
- **解決方案**：
  - 更新 `CyclicFrameCodec` 的 `extract_ry_rww()` 與 `build_slmp_0e70_profile_response()`，全面對齊實機 143B 請求解析與 131B `wfirst` 回包結構，實機接上後 FX5U 立即穩定完成 CSR/SSR 握手並鎖定掃描週期。

---

### 問題 3：SLMP `4020h` 非週期參數讀寫出現長度錯誤或通訊無回應
- **問題現象**：
  - FX5U 運動控制 FB（如寫入 `6098h`、`6099h` 或讀取參數）透過 SLMP `4020h` 存取物件字典時，遇到讀取長度對齊異常或無法連線。
- **根因分析**：
  1. 實機部分運動功能塊會透過 **TCP Port 5010** 而非僅 UDP 5010 發送 SLMP `4020h` 請求。
  2. SLMP `4020h` Read 回應中，`Number of Data Value` 與偶數位元組對齊規則若計算錯誤會導致主站解析失敗；且寫入 `2006h`/`2007h`（PA06/PA07 電子齒輪比）或偶數補零 (`pad`) 時需要嚴謹處理。
- **解決方案**：
  - 在 `slmp_server.py` 同時啟動 **UDP Port 5010 與 TCP Port 5010** (`asyncio.start_server`)，並於 `SLMPFrameProcessor` 處理 TCP 串流拆包與 SLMP 3E/4E 回應組裝。
  - 修正 `_handle_4020()` 讀取回應之實際 `num_data_value` 與資料長度組裝，並在 `ObjectDictionary.slmp_write_object()` 補齊標準單筆/區塊讀寫驗證邏輯。

---

### 問題 4：PP/PV 速度換算比例不符，以及 `2D20h` 意外限制速度與 `2D11h`/`2D12h` 為 0
- **問題現象**：
  1. 執行 `MC_MoveRelative` 或 `MC_MoveVelocity` 時，位置增量與設定的 `10,000` 單位速度不符。
  2. 在 PV 運轉中速度被意外截斷為 `0` 或受限。
  3. 觀察 `RWrn02`/`RWrn03`（對應 `2D11h` S D01 / `2D12h` S D02）數值在 PV 加速或運轉時皆為 `0`（而實機監看 `W2=4096 / W3=8` 等位元會隨狀態變化）。
- **根因分析**：
  1. 士林 SDC-F 預設採用 **24-bit 編碼器 (`608Fh:01 = 16,777,216 pulse/rev`)** 且預設電子齒輪比 **`PA06/PA07 = 16,777,216 / 10,000`**（即 PLC 端每 `10,000` 指令脈波對應馬達轉 1 圈，內部物理位置與指令以脈波比率換算）。模擬器原本固定以 `10,000` 或未正確套用 `encoder_ppr` 與 `2006h/2007h` 換算導致速度失真。
  2. 依據 SDC-F Mode 1 (`RWwn0A~0B`) 規格，**`2D20h` (Velocity limit value) 僅在 PT（Profile Torque，扭矩控制模式）下生效**。當 PLC 未寫入 `2D20h`（為 0）且處於 PP/PV 模式時，若模擬器拿 `2D20h` 當作所有模式的速度上限會把速度誤鎖為 0。
  3. `2D11h` 與 `2D12h` 屬於驅動器即時狀態 DO（如 Bit 2 `S_SA` 速度到達、Bit 12 `S_INP` 定位完成、Bit 13 `S_TLC` 扭矩限制、Bit 3 `S_ZSP` 零速檢出、Bit 4 `S_VLC` 速度限制），需要依 Servo ON 狀態、當前轉速與誤差即時計算。
- **解決方案**：
  - 於 `object_dictionary.py` 將 `608Fh:01` 與 `2006h` 預設值調整為實機標準 `16777216`、`2007h` 為 `10000`，並在 `servo_axis.py` 實作精確的速度／脈波／電子齒輪比物理積分。
  - 限定 `2D20h` 速度限制 **僅在 `OpMode.PT` 時套用**，PV/PP 則依 `607Fh`/`6080h` 限制。
  - 在 `servo_axis.py` 補齊 `2D11h`（含 PV 模式到達目標速度即輸出 `S_SA`、PP 定位完成輸出 `S_INP`）與 `2D12h`（依 `606Fh` 零速門檻計算 `S_ZSP` 與 `S_VLC`）之即時位元更新邏輯。

---

### 問題 5：三菱官方 `MC_MoveVelocity_CCLinkIEFBasic_F` 與非官方 `MC_Jog` 跳出 `Error 4611 (0x1203)` 且 `6060h` 維持 `1`
- **問題現象**：
  - 相同的 PLC 程式接**實體 SDC-F 伺服**時，執行三菱官方 `MC_MoveVelocity_CCLinkIEFBasic_F` 與非官方／自訂 `MC_Jog` 功能塊完全正常；但改接模擬器時，一旦觸發執行，`Error` 立即亮起並報錯 **`ErrorID = 4611 (0x1203)`**。
  - 檢查 SLMP Port 5010 日誌發現 **Port 5010 完全沒被查詢**，且從 `MC_MoveRelative` 切換執行 `MC_MoveVelocity` 時，`RWwn00` (`6060h`) 也沒有變為 `3` 而是維持在 `1`。
- **根因定位（實機對比決定性突破）**：
  - 比對使用者提供的實體 SDC-F 伺服與模擬器在 GX Works3「軟元件批次監視」中的 `B0~B30`（`RX` 位元區）與 `W0~W11`（`RWr` 字元區）畫面截圖：
    - 模擬器當時在 `SSR=1` 時僅點亮 `RX3F`（即 `B30` 顯示 `-32768` / `0x8000` -> `rx[7] = 0x80`）。
    - **實體 SDC-F 伺服** 在上線就緒時的 `B30` 顯示 **`-16384`（即 `0xC000` 二進位 `1100 0000 0000 0000`）**，代表 **同時點亮了 `RX3F (B3F)` 與 `RX3E (B3E)`**！
  - 無論是三菱官方 `MC_MoveVelocity_CCLinkIEFBasic_F` 或是非官方／自訂的 `MC_Jog` 功能塊，在送出切換 `6060h` 或啟動指令前，皆會先檢查遠端站的 `RX3E (B3E)`（Remote Ready / 初期設定完成就緒位元）是否為 ON。由於模擬器缺少 `RX3E`，FB 在第一掃描週期便直接判定驅動器通訊/初期設定未就緒，立即中止並不寫入 `6060h`（直接拋出 Error `4611`），也不會往下查詢 Port 5010。
- **解決方案**：
  - 修正 `ServoAxis._build_rx_bytes()`（並移除舊版不完整旗標切換）：當 `self.ssr_active` 為 True 時，將 `rx[7]` 設為 **`0xC0`**（同時拉高 **Bit 7 = `RX3F` SSR** 與 **Bit 6 = `RX3E` Remote Ready**）；發生異警時再拉高 **Bit 3 = `RX3B`**（`0xC8`）。
  - **驗證結果**：重新對接實體 FX5U PLC 測試，`B30` 顯示為 `-16384`，官方 `MC_MoveVelocity_CCLinkIEFBasic_F` 與非官方 `MC_Jog` 皆順利切換 `6060h = 3` 並正常運轉，`Error` 為 OFF (`ErrorID = 0`)，馬達位置與速度完全正常響應。

---

### 問題 6：多站含保留站 (`0.0.0.0`) 與 `SLMP 4020h` 直接寫入 `1A00h:10` 回報 `0xCCC7`
- **問題現象**：
  - 當 PLC 組態為 4 軸 + 1 I/O 站（總站數 5，其中 3 軸設為保留站 `0.0.0.0`）時，模擬器在 `CSR=0` 階段收到 `Write 1A00h:10` 並回報 `EndCode=0xCCC7`，導致無法建立 `CSR=1` 連線。
- **根因分析**：
  - PLC 於 `CSR=0` 期間並未先寫入 `1A00h:00 = 0`，即透過 `4020h` 直接修改 `1A00h:10` 映射項目；模擬器原規則誤將 `SubIndex 0 != 0` 一併擋下。
  - 多站 `0E70h` 封包內含各站 Slave IP 表（保留站填 `0.0.0.0`），廣播監聽分派若以順序索引覆蓋 IP 比對，會誤讓保留站搶佔槽位。
- **解決方案與驗證**：
  - `object_dictionary.py` 放寬為僅在 `CSR=1` (`pdo_locked=True`) 時禁止修改 `1600h`/`1A00h`；`CSR=0` 時允許直接寫入並自動重建 PDO 映射。
  - `iefb_cyclic_server.py` 改以 `0E70h` Slave IP 表精確比對為最優先，保留站 (`0.0.0.0`) 靜默略過。
  - **驗證結果**：以「4 軸（含 3 軸保留站）+ 1 未連線 I/O 站」架構實機連線測試，軸 1 成功建立循環連線且 `MC_MoveRelative` 動作完全正常。

---

## 三、 目前驗證狀態與結論

經過上述六個階段的深度對接與除錯，目前模擬器在無需更改任何三菱 PLC 程式的前提下，已達成與實體士林電機 SDC-F 伺服高度一致的通訊與運動行為：

- **CC-Link IEFB 連線探索與週期交換**：`0E30h`、`0E31h`、`45237`、`0E70h` (131B `wfirst`，含多站與保留站 `0.0.0.0` 過濾)、`RX3F+RX3E` (`0xC0`) 全面穩定。
- **非週期參數存取**：UDP / TCP `5010` SLMP `4020h` 讀寫與塊傳送正常運作（含 `CSR=0` 下直接寫入 `1600h`/`1A00h` 映射項）。
- **運動控制功能塊**：
  - **三菱官方 FB**：`MC_Power`、`MC_Home`、`MC_MoveRelative`、`MC_MoveVelocity_CCLinkIEFBasic_F`、`MC_Stop`、`MC_Reset` 均驗證動作正常。
  - **非官方／自訂 FB**：`MC_Jog`（使用者自訂／非三菱官方 JOG 功能塊）亦驗證動作正常。
