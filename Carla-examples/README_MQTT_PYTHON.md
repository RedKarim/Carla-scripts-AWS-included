# Python MQTT Broker Setup

## 概要

Mosquittoをインストールする必要はありません！PythonベースのMQTTブローカーを使用できます。

## クイックスタート

### 方法1: バッチファイルを使用（Windows）

```cmd
start_mqtt_broker_python.bat
```

これで自動的に必要なライブラリをインストールしてMQTTブローカーを起動します。

### 方法2: 手動で起動

1. **必要なライブラリをインストール:**
   ```cmd
   pip install hbmqtt
   ```

2. **MQTTブローカーを起動:**
   ```cmd
   python mqtt_broker.py
   ```

## 動作確認

MQTTブローカーが起動すると、以下のメッセージが表示されます：

```
============================================================
MQTT Broker started successfully!
============================================================
Broker running on: 127.0.0.1:1883
Press Ctrl+C to stop the broker
============================================================
```

## CARLAスクリプトの実行

MQTTブローカーを起動した状態で、別のターミナルでCARLAスクリプトを実行：

```cmd
python manual_control.py
```

または

```cmd
python manual_control_steeringwheel.py
```

以下のメッセージが表示されれば成功です：

```
MQTT接続成功: localhost:1883
MQTT購読接続成功: localhost:1883
MQTT購読開始: carla/leader/state
```

## トラブルシューティング

### "hbmqtt library not installed" エラー

**解決策:**
```cmd
pip install hbmqtt
```

### "Python is not installed" エラー

**解決策:** Python 3.6以上をインストールしてください。

### ポート1883が既に使用されている

**解決策:** 他のMQTTブローカー（Mosquittoなど）が既に起動している可能性があります。
- 既存のMQTTブローカーを停止する
- または、`mqtt_broker.py`のポート番号を変更する

### 接続できない

**確認事項:**
1. MQTTブローカーが起動しているか確認
2. ファイアウォールがポート1883をブロックしていないか確認
3. `mqtt_follower.py`の`MQTT_BROKER`設定が`"localhost"`になっているか確認

## 利点

- ✅ Mosquittoのインストール不要
- ✅ 純粋なPython実装
- ✅ 簡単に起動・停止
- ✅ クロスプラットフォーム（Windows/Mac/Linux）

## 停止方法

MQTTブローカーを停止するには、実行中のターミナルで `Ctrl+C` を押してください。
