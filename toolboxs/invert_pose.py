from scipy.spatial.transform import Rotation as R

def invert_poses(input_file, output_file):
    with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
        for line in f_in:
            if line.startswith('#'):
                f_out.write(line)
                continue

            line = line.strip()
            if not line:
                continue  # Skip empty lines
            tokens = line.split()
            if len(tokens) < 4:
                f_out.write(line + '\n')
                continue

            # Parse the input values
            # idx, QW, QX, QY, QZ, TX, TY, TZ, camera_idx, file_name = map(float, tokens)
            idx = int(tokens[0])
            QW = float(tokens[1])
            QX = float(tokens[2])
            QY = float(tokens[3])
            QZ = float(tokens[4])
            TX = float(tokens[5])
            TY = float(tokens[6])
            TZ = float(tokens[7])
            camera_idx = int(tokens[8])
            file_name = tokens[9]

            # Construct the quaternion in [x, y, z, w] format
            quat = [QX, QY, QZ, QW]
            # Create the rotation object
            rotation = R.from_quat(quat)
            # Invert the rotation
            rotation_inv = rotation.inv()
            # Invert the translation
            translation = [TX, TY, TZ]
            translation_inv = -rotation_inv.apply(translation)
            # Get the inverted quaternion components
            quat_inv = rotation_inv.as_quat()  # Returns [x, y, z, w]
            # Rearrange to [QW, QX, QY, QZ] format
            QW_inv = quat_inv[3]
            QX_inv = quat_inv[0]
            QY_inv = quat_inv[1]
            QZ_inv = quat_inv[2]
            # Prepare the output line
            output_line = f"{idx} {QW_inv} {QX_inv} {QY_inv} {QZ_inv} {translation_inv[0]} {translation_inv[1]} {translation_inv[2]} {camera_idx} {file_name}\n"
            f_out.write(output_line)

if __name__ == '__main__':
    data_dir = 'synced_data/Orientations-1/synced_202411201946/sparse/0'
    invert_poses(f'{data_dir}/keyframes.txt', f'{data_dir}/images.txt')