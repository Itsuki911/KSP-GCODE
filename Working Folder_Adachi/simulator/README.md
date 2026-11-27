# KSP-GCODE Local OSS Simulator (`Working Folder_Adachi/simulator`)

ローカル環境で **KSP 回転塗布機（X=ターンテーブル回転角 deg, Y=半径リニア mm, Z=ノズル昇降 mm, ボールバルブサーボ）** の動作を検証するための OSS シミュレータ環境です。

## 構成ファイル

- `klipper_sim.py` — リポジトリ内の Klipper 実コード（`GCodeMove`, `Move`, `LookAheadQueue`）と `printer_data/config/printer.cfg` の設定をそのまま読み込み、台形速度計画・ホーミング・待機・円筒/極座標変換・実表面周速を計算。
- `workpiece_model.py` — `Worker-Image.jpeg` を参考に座標・高さ整合を取って新規設計したパラメトリック 3D ワークモデル（鋳物本体、中央加工リング、外周5ボルトボス C1〜C5、段差パッド Raised 1/2、青色パージパッド）。軌道の視認性を損なわない半透明＋輪郭線設計。
- `ksp_machine.xml` — MuJoCo 3D 機構モデル（ターンテーブル回転軸 X + 半径スライド Y + 昇降ヘッド Z + ボールバルブサーボレバー）。
- `mujoco_sim.py` — MuJoCo によるヘッドレス衝突・最小クリアランス（床面＋ワーク各部位との `mj_geomDistance`）・逆動力学トルク/推力計算、および 3D ワーク＋軌道線を同時描画するリアルタイム 3D GUI ビューワ。
- `visualize.py` — 単一 HTML 完結型（ダウンロードして開くだけで動作）の Three.js 3D マルチビューアニメーション HTML（**実機モード：ワーク回転＋ノズル Y/Z 移動** ⇔ **ワーク固定モード** 切替、斜め/真上/真横/正面/ノズル近接カメラ、3D+真上+真横の同時多視点表示、同期2Dテレメトリ、下部コントロール＆凡例の分離配置）、Matplotlib PNG サマリ、一覧ダッシュボード `output/index.html` を生成。
- `run_sim.py` — 統合 CLI エントリポイント（`--no-workpiece` でワーク非表示も可）。

## 使い方

### 1. 全単一モーションテスト (`single_motion_tests/*.gcode`) + `testPaint*.gcode` を一括シミュレーション
```bash
cd /Users/adachiitsuki/KSP-GCODE
python3 "Working Folder_Adachi/simulator/run_sim.py" --all
open "Working Folder_Adachi/simulator/output/index.html"
```

### 2. 任意の1ファイルを MuJoCo 3D ビューワでリアルタイム再生（macOS は `mjpython` を使用）
```bash
cd /Users/adachiitsuki/KSP-GCODE
mjpython "Working Folder_Adachi/simulator/run_sim.py" \
  "Working Folder_Adachi/single_motion_tests/13_clearance_check_high_z.gcode" \
  --view --speed 2.0
```
`testPaint.gcode` の全工程を見る場合：
```bash
mjpython "Working Folder_Adachi/simulator/run_sim.py" \
  "Working Folder_Adachi/testPaint.gcode" \
  --view --speed 2.0 --loop
```

### 3. ワーク / 治具の STL メッシュを載せて衝突・クリアランス判定する場合
```bash
python3 "Working Folder_Adachi/simulator/run_sim.py" \
  "Working Folder_Adachi/single_motion_tests/13_clearance_check_high_z.gcode" \
  --workpiece-stl /path/to/workpiece.stl --workpiece-scale 0.001
```

### 4. バルブ応答遅れ（例: 250 ms）や回転中心オフセットを仮定して検証する場合
```bash
python3 "Working Folder_Adachi/simulator/run_sim.py" \
  "Working Folder_Adachi/testPaint.gcode" \
  --valve-response-ms 250 --r-center 180.0
```

## 注意事項（実機検証との切り分け）
- 本シミュレータは Klipper の軌道計画・円筒/極座標運動学・3D幾何干渉を検証するものであり、TMC2209 電気異常（`s2vsa`）、StealthChop 実負荷脱調、リミットスイッチ配線極性、実液体の流体挙動は代替しません。
- 元の `gcodes/*.gcode`、`printer_data/`、`Working Folder_Adachi/testPaint.gcode` は一切変更しません。

### エンドストップ入力確認のシミュレーション

`00_endstop_sensor_check.gcode` は `--all` に含まれます。個別実行も可能です。

```bash
python3 "Working Folder_Adachi/simulator/run_sim.py" "Working Folder_Adachi/single_motion_tests/00_endstop_sensor_check.gcode"
```

待機とメッセージ・読み取り時刻を再現しますが、実センサーの遮光、入力レベル、配線、合否はシミュレーションできません。`QUERY_ENDSTOPS` の結果は不明として警告します。実機のコンソールで確認してください。

### Inner円環上のシミュレーション候補

`testPaint_inner_ring_simulation.gcode` は `--all` に含まれ、元の `testPaint.gcode` と別のHTMLに表示されます。innerのY座標のみを `Y180` から `Y131` に変更しています。既定の回転中心 `r_center=180 mm` と、モデルの円環半径 `40..58 mm` の仮定では、半径49 mmの円環中央を通ります。形状、Y原点、ローラーの接触幅とZ高さは未実測であり、実機への使用は校正まで行わないでください。シミュレーターは実際の塗膜やローラー接触を検証しません。

```bash
python3 "Working Folder_Adachi/simulator/run_sim.py" "Working Folder_Adachi/testPaint_inner_ring_simulation.gcode" --no-mujoco
```

### 坂付きpurge・inner修正版

`testPaint_finalReviced_simulation.gcode` は7度の仮想坂をX/Zで9往復し、innerではY131を使用します。独立した `14_purge_7deg_simulation_only.gcode` と合わせて `--all` に含まれます。青いpurge台は平面のままで、ローラー接触の検証はできません。

`testPaint_finalReviced_machine.gcode` は同じ実行命令を持つマシン用候補ですが、寸法・原点・ローラー接触・干渉・エンドストップ・バルブ動作は未確認です。実機実行前に検証が必要です。重複表示を避けるため `--all` から除外します。
