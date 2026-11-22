# KSP-GCODE Local OSS Simulator (`Working Folder_Adachi/simulator`)

ローカル環境で **KSP 回転塗布機（X=ターンテーブル回転角 deg, Y=半径リニア mm, Z=ノズル昇降 mm, ボールバルブサーボ）** の動作を検証するための OSS シミュレータ環境です。

## 構成ファイル

- `klipper_sim.py` — リポジトリ内の Klipper 実コード（`GCodeMove`, `Move`, `LookAheadQueue`）と `printer_data/config/printer.cfg` の設定をそのまま読み込み、台形速度計画・ホーミング・待機・円筒/極座標変換・実表面周速を計算。
- `workpiece_model.py` — `Worker-Image.jpeg` を参考に座標・高さ整合を取って新規設計したパラメトリック 3D ワークモデル（鋳物本体、中央加工リング、外周5ボルトボス C1〜C5、段差パッド Raised 1/2、青色パージパッド）。軌道の視認性を損なわない半透明＋輪郭線設計。
- `ksp_machine.xml` — MuJoCo 3D 機構モデル（ターンテーブル回転軸 X + 半径スライド Y + 昇降ヘッド Z + ボールバルブサーボレバー）。
- `mujoco_sim.py` — MuJoCo によるヘッドレス衝突・最小クリアランス（床面＋ワーク各部位との `mj_geomDistance`）・逆動力学トルク/推力計算、および 3D ワーク＋軌道線を同時描画するリアルタイム 3D GUI ビューワ。
- `visualize.py` — 半透明 3D ワーク＋高視認性軌道線の Plotly 3D アニメーション HTML（凡例クリックでワーク表示 ON/OFF 切替可能）、Matplotlib PNG サマリ、一覧ダッシュボード `output/index.html` を生成。
- `run_sim.py` — 統合 CLI エントリポイント（`--no-workpiece` でワーク非表示も可）。

## 使い方

### 1. 全16本 (`single_motion_tests/*.gcode`) + `testPaint.gcode` を一括シミュレーション
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
