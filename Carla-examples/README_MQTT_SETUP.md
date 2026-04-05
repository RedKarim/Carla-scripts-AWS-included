# MQTT Broker Setup Guide for Windows

## クイックスタート

### 最も簡単な方法

1. **`start_mqtt_broker_admin.bat`** を右クリック
2. **「管理者として実行」** を選択
3. 完了！

## 詳細な手順

### ステップ1: 管理者権限でコマンドプロンプトを開く

**方法A: スタートメニューから**
1. Windowsキーを押す
2. "cmd" または "Command Prompt" を検索
3. 右クリックして **「管理者として実行」** を選択

**方法B: ファイルエクスプローラーから**
1. このフォルダを開く
2. `start_mqtt_broker_admin.bat` を右クリック
3. **「管理者として実行」** を選択

### ステップ2: Mosquittoサービスをインストール（初回のみ）

管理者のコマンドプロンプトで：

```cmd
cd "C:\Program Files\Mosquitto"
mosquitto.exe install
```

**注意**: もし "C:\Program Files\Mosquitto" が見つからない場合：
- "C:\Program Files (x86)\Mosquitto" を試してください
- または、Mosquittoがインストールされている実際のパスを使用してください

### ステップ3: サービスを起動

```cmd
net start mosquitto
```

### ステップ4: 動作確認

別のコマンドプロンプト（通常権限でOK）で：

```cmd
mosquitto_sub -h localhost -t "test" -v
```

別のターミナルで：

```cmd
mosquitto_pub -h localhost -t "test" -m "Hello MQTT"
```

最初のターミナルに "Hello MQTT" が表示されれば成功です！

## トラブルシューティング

### "Access is denied" エラー

**解決策**: 管理者権限で実行してください
- コマンドプロンプトを管理者として開く
- または `start_mqtt_broker_admin.bat` を管理者として実行

### "System error 5 has occurred"

**解決策**: これも管理者権限が必要です
- コマンドプロンプトを管理者として開き直してください

### "The service name is invalid"

**解決策**: サービスがインストールされていません
```cmd
cd "C:\Program Files\Mosquitto"
mosquitto.exe install
net start mosquitto
```

### サービスが起動しない

サービス状態を確認：
```cmd
sc query mosquitto
```

エラーログを確認：
```cmd
eventvwr.msc
```
Windowsログ > アプリケーション でMosquittoのエラーを確認

### Mosquittoが見つからない

インストール場所を確認：
```cmd
where mosquitto
```

または、Program Filesフォルダを確認：
```cmd
dir "C:\Program Files" | findstr /i mosquitto
dir "C:\Program Files (x86)" | findstr /i mosquitto
```

## サービス管理コマンド

### サービスを停止
```cmd
net stop mosquitto
```

### サービスを再起動
```cmd
net stop mosquitto
net start mosquitto
```

### サービスを削除（アンインストール）
```cmd
sc delete mosquitto
```

### サービス状態を確認
```cmd
sc query mosquitto
```

## 自動起動の設定

MosquittoをWindows起動時に自動で開始するには：

1. `Win + R` を押す
2. `services.msc` と入力してEnter
3. "mosquitto" を探す
4. 右クリック > プロパティ
5. 「スタートアップの種類」を「自動」に設定

## 確認方法

MQTTブローカーが正常に動作している場合、CARLAスクリプトを実行すると以下のメッセージが表示されます：

```
MQTT接続成功: localhost:1883
MQTT購読接続成功: localhost:1883
MQTT購読開始: carla/leader/state
```

これらのメッセージが表示されれば、MQTTモードで動作しています！
