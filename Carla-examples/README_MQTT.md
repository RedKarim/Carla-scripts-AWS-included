# MQTT Follower Vehicle Setup

## 現在の動作

フォロワー車両は**直接フォローモード**で正常に動作しています。MQTTが利用できない場合でも、リーダー車両を追従します。

## MQTT接続を有効にする方法

### Windowsでのセットアップ

#### 1. Mosquittoのインストール

1. https://mosquitto.org/download/ からWindows版をダウンロード
2. インストーラーを実行してインストール
3. インストール時に「サービスとしてインストール」を選択することを推奨

#### 2. MQTTブローカーの起動

**方法A: サービスとして起動（推奨）**
```cmd
net start mosquitto
```

**方法B: コマンドラインから起動**
```cmd
mosquitto -c mosquitto.conf
```

**方法C: 提供されたバッチファイルを使用**
```cmd
start_mqtt_broker.bat
```

#### 3. 接続確認

別のターミナルで以下を実行して接続を確認：
```cmd
mosquitto_sub -h localhost -t "carla/leader/state" -v
```

### Linux/Macでのセットアップ

#### 1. Mosquittoのインストール

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install mosquitto mosquitto-clients
```

**macOS:**
```bash
brew install mosquitto
```

#### 2. MQTTブローカーの起動

```bash
mosquitto -c /etc/mosquitto/mosquitto.conf
```

または、サービスとして起動：
```bash
sudo systemctl start mosquitto  # Linux
brew services start mosquitto   # macOS
```

## 動作モード

### 直接フォローモード（MQTT不要）
- リーダー車両の状態をCARLA APIから直接取得
- 同じプロセス内で動作
- ネットワーク不要

### MQTTモード（ネットワーク経由）
- リーダー車両の状態をMQTT経由で送信
- 異なるマシン/プロセス間で動作可能
- ネットワーク経由での通信

## 設定

`mqtt_follower.py`の以下の定数を変更して設定をカスタマイズできます：

```python
MQTT_BROKER = "localhost"  # MQTTブローカーのアドレス
MQTT_PORT = 1883            # MQTTブローカーのポート
FOLLOWER_DISTANCE = 10.0     # フォロワー車両との目標距離（メートル）
```

## トラブルシューティング

### MQTT接続エラーが表示される

- **問題**: `[WinError 10061] No connection could be made`
- **解決策**: MQTTブローカーが起動していません。上記の手順で起動してください。
- **注意**: エラーが表示されても、直接フォローモードで動作するため問題ありません。

### フォロワー車両が動かない

- リーダー車両が存在することを確認
- フォロワー車両が正常にスポーンされていることを確認（IDが表示される）
- コンソールにエラーメッセージがないか確認

## テスト

MQTTが正常に動作している場合、以下のメッセージが表示されます：

```
MQTT接続成功: localhost:1883
MQTT購読接続成功: localhost:1883
MQTT購読開始: carla/leader/state
```

これらのメッセージが表示されない場合でも、直接フォローモードで動作します。
