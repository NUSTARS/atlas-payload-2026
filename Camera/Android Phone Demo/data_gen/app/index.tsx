import { CameraView, CameraType, useCameraPermissions } from "expo-camera";
import { DeviceMotion } from "expo-sensors";
import * as FileSystem from "expo-file-system/legacy";
import * as Sharing from "expo-sharing";
import { useRef, useState, useEffect } from "react";
import { Button, StyleSheet, Text, TouchableOpacity, View, Alert } from "react-native";

const TARGET_FPS = 30;                 // desired logging/display rate
const SAMPLE_MS = 1000 / TARGET_FPS;   // sampling interval for orientation logging

export default function App() {
  const [facing, setFacing] = useState<CameraType>("back");
  const [permission, requestPermission] = useCameraPermissions();
  const camRef = useRef<CameraView | null>(null);
  const [recording, setRecording] = useState(false);

  // orientation state
  const oriRowsRef = useRef<string[]>([]);
  const oriSubRef = useRef<any>(null);
  const [lastOri, setLastOri] = useState({ roll: 0, pitch: 0, yaw: 0 }); // radians
  const samplerRef = useRef<any>(null);

  useEffect(() => {
    // run device motion at a high rate; we downsample to TARGET_FPS
    DeviceMotion.setUpdateInterval(5);
  }, []);

  if (!permission) return <View />;
  if (!permission.granted) {
    return (
      <View style={styles.container}>
        <Text style={styles.message}>We need your permission to show the camera</Text>
        <Button onPress={requestPermission} title="Grant permission" />
      </View>
    );
  }

  const toggleCameraFacing = () =>
    setFacing(cur => (cur === "back" ? "front" : "back"));

  // --- Orientation capture (Euler angles from DeviceMotion.rotation) ---
  const startOrientationLogging = () => {
    oriRowsRef.current = ["frame_idx,timestamp_ms,roll_rad,pitch_rad,yaw_rad"];
    let frame = 0;
    let latest = { roll: 0, pitch: 0, yaw: 0 };

    // Subscribe to device attitude (Euler angles, radians)
    oriSubRef.current = DeviceMotion.addListener(({ rotation }) => {
      if (!rotation) return;
      // Expo gives rotation as alpha(Z), beta(X), gamma(Y)
      const yaw = rotation.alpha ?? 0;   // about Z
      const pitch = rotation.beta ?? 0;  // about X
      const roll = rotation.gamma ?? 0;  // about Y
      latest = { roll, pitch, yaw };
      setLastOri(latest);
    });

    // Downsample and log at TARGET_FPS
    samplerRef.current = setInterval(() => {
      const t = Date.now();
      oriRowsRef.current.push(
        `${frame++},${t},${latest.roll},${latest.pitch},${latest.yaw}`
      );
    }, SAMPLE_MS);
  };

  const stopOrientationLogging = () => {
    oriSubRef.current?.remove?.();
    oriSubRef.current = null;
    clearInterval(samplerRef.current);
    samplerRef.current = null;
  };

  // --- Recording control ---
  const startRecording = async () => {
    if (!camRef.current || recording) return;
    setRecording(true);
    startOrientationLogging();

    try {
      const video = await camRef.current.recordAsync({
        maxDuration: 300,
        mute: true,
        quality: "1080p",
        frameRate: TARGET_FPS, // hint; device-dependent
      });

      // write CSV
      const csvUri = `${FileSystem.documentDirectory}orientation_${Date.now()}.csv`;
      const csvData = oriRowsRef.current.join("\n");
      await FileSystem.writeAsStringAsync(csvUri, csvData, {
        encoding: FileSystem.EncodingType.UTF8,
      });

      // share both files
      if (await Sharing.isAvailableAsync()) {
        await Sharing.shareAsync(video.uri);
        await Sharing.shareAsync(csvUri);
      } else {
        Alert.alert("Files saved", `Video: ${video.uri}\nCSV: ${csvUri}`);
      }
    } catch (e) {
      console.error(e);
      Alert.alert("Recording failed", String(e));
    } finally {
      stopOrientationLogging();
      setRecording(false);
    }
  };

  const stopRecording = () => {
    if (!camRef.current || !recording) return;
    camRef.current.stopRecording();
  };

  // Helper to show degrees in the HUD (internal values remain radians)
  const toDeg = (rad: number) => (rad * 180) / Math.PI;

  return (
    <View style={styles.container}>
      <CameraView
        ref={camRef}
        style={styles.camera}
        facing={facing}
        mode="video"
        audio={false}
        videoQuality="1080p"
        frameRate={TARGET_FPS}
      />
      <View style={styles.overlay}>
        <Text style={styles.gyroText}>
          roll:{toDeg(lastOri.roll).toFixed(1)}°  pitch:{toDeg(lastOri.pitch).toFixed(1)}°  yaw:{toDeg(lastOri.yaw).toFixed(1)}° @ {TARGET_FPS} fps
        </Text>
      </View>
      <View style={styles.buttonContainer}>
        <TouchableOpacity style={styles.button} onPress={toggleCameraFacing}>
          <Text style={styles.text}>Flip</Text>
        </TouchableOpacity>

        {!recording ? (
          <TouchableOpacity style={styles.button} onPress={startRecording}>
            <Text style={styles.text}>Start</Text>
          </TouchableOpacity>
        ) : (
          <TouchableOpacity style={styles.button} onPress={stopRecording}>
            <Text style={styles.text}>Stop</Text>
          </TouchableOpacity>
        )}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, justifyContent: "center", backgroundColor: "#000" },
  message: { textAlign: "center", paddingBottom: 10, color: "#fff" },
  camera: { flex: 1 },
  overlay: { position: "absolute", top: 40, left: 20, right: 20 },
  gyroText: { color: "#fff", fontSize: 14, opacity: 0.8 },
  buttonContainer: {
    position: "absolute",
    bottom: 64,
    flexDirection: "row",
    width: "100%",
    paddingHorizontal: 64,
    justifyContent: "space-between",
  },
  button: { flex: 1, alignItems: "center", paddingVertical: 10 },
  text: { fontSize: 22, fontWeight: "bold", color: "white" },
});
