from PIL import Image
import cv2
import os
import csv
import math

class Scanner:
    output_folder = "images/"

    def import_images(self, video_path):
        print("Images Imported")

        cap = cv2.VideoCapture(video_path)

        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)

        if not cap.isOpened():
            print(f"Error: Could not open video file {video_path}")
            return

        frame_count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            image_filename = os.path.join(self.output_folder, f"frame_{frame_count:05d}.png")
            cv2.imwrite(image_filename, frame)
            frame_count += 1

        cap.release()

        return [
            Image.open(os.path.join(self.output_folder, f))
            for f in sorted(os.listdir(self.output_folder))
            if f.endswith(".png")
        ]


    def import_csv(self, orientation_path):
        output = []

        print("Orientation data imported")
        with open(orientation_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            next(reader, None)  

            for row in reader:
                # convert each value in the row to float
                try:
                    floats = [float(v) for v in row[:3]]  # keep only first 3 columns
                    output.append(floats)
                except ValueError:
                    # skip any malformed lines
                    continue

        return output
    

    def __init__(self, video_path, orientation_path):
        # Load all saved images as PIL Images
        self.images = self.import_images(video_path)

        # Load orientation data from the CSV file
        self.orientation = self.import_csv(orientation_path)

        self.length = min(len(self.images), len(self.orientation))

        if self.length == 0:
            raise RuntimeError("No frames or orientation rows found.")
        if len(self.frame_paths) != len(self.orientation):
            print(f"Warning: frames ({len(self.frame_paths)}) and IMU rows ({len(self.orientation)}) differ; truncating to {n}.")

        # ensure both are the same size
        self.frame_paths = self.frame_paths[:self.length]
        self.orientation = self.orientation[:self.length]

    
    @staticmethod
    def normalize(vec):
        x, y, z = vec
        n = math.sqrt(x*x + y*y + z*z)
        if n == 0: return (0.0, 0.0, 0.0), 0.0
        return (x/n, y/n, z/n), n


    @staticmethod
    def angle_to_target(look_vec, target_dir=(0.0, 0.0, 0.0)):
        u, nu = Scanner.normalize(look_vec)
        t, nt = Scanner.normalize(target_dir)
        if nu == 0 or nt == 0:
            return 180.0
        dot = u[0]*t[0] + u[1]*t[1] + u[2]*t[2]
        dot = max(min(dot, 1.0), -1.0)
        return math.degrees(math.acos(dot))


    def create_2d(self,
                  max_angle_deg=5.0,
                  out_path="flat_only.mp4",
                  keep_top_percent=None,
                  target_dir=(0.0, 0.0, -1.0),
                  auto_down=True):

        if auto_down:
            # Compare average dot product to (0,0,1) vs (0,0,-1)
            z_pos = (0.0, 0.0,  1.0)
            z_neg = (0.0, 0.0, -1.0)
            dots_pos, dots_neg = [], []
            for v in self.orientation:
                up,_ = self._norm(v)
                dots_pos.append(up[0]*z_pos[0] + up[1]*z_pos[1] + up[2]*z_pos[2])
                dots_neg.append(up[0]*z_neg[0] + up[1]*z_neg[1] + up[2]*z_neg[2])
            mean_pos = sum(dots_pos)/len(dots_pos)
            mean_neg = sum(dots_neg)/len(dots_neg)
            target_dir = z_neg if mean_neg >= mean_pos else z_pos
            print(f"Auto-selected target_dir = {target_dir} (mean dot: +Z={mean_pos:.3f}, -Z={mean_neg:.3f})")

        # Angle of each frame’s look vector to target_dir
        angles = [self._angle_to_target(v, target_dir) for v in self.orientation]

        # Choose indices to keep
        if keep_top_percent is not None:
            assert 0 < keep_top_percent <= 1.0
            order = sorted(range(len(angles)), key=lambda i: angles[i])
            k = max(1, int(len(order) * keep_top_percent))
            keep = set(order[:k])
            indices = [i for i in range(len(angles)) if i in keep]
            print(f"Keeping top {keep_top_percent*100:.1f}% flattest frames: {len(indices)} frames.")
        else:
            indices = [i for i,a in enumerate(angles) if a <= max_angle_deg]
            print(f"Keeping frames with angle ≤ {max_angle_deg}°: {len(indices)} frames.")

        if not indices:
            print("No frames met the nadir criterion. Try a larger max_angle_deg or use keep_top_percent.")
            return

        # Write output video
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(out_path, fourcc, self.fps, (self.width, self.height))
        if not writer.isOpened():
            raise RuntimeError(f"Could not open VideoWriter at {out_path}")

        for i in indices:
            frame_bgr = cv2.imread(self.frame_paths[i], cv2.IMREAD_COLOR)
            if frame_bgr is None:
                continue
            # Optional overlay for debugging
            # cv2.putText(frame_bgr, f"{angles[i]:.1f} deg", (16,40),
            #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2, cv2.LINE_AA)
            writer.write(frame_bgr)

        writer.release()
        print(f"Wrote {len(indices)} frames to {out_path} at ~{self.fps:.2f} fps.")


        
