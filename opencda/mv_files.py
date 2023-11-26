import os, shutil

def main():

    src_dir = '/media/hdd1/opv2v/opencda_dump/train/'
    des_dir = '/media/hdd1/opv2v/opencda_dump/test/'
    rang = [292, 315]
    interval = 2

    exts = ['_camera0.png', '.yaml', '.pcd']

    for scene_dir in  os.listdir(src_dir):

        for rsu_dir in os.listdir(os.path.join(src_dir, scene_dir)):
            
            des_path = os.path.join(des_dir, scene_dir, rsu_dir)
            
            if not os.path.exists(des_path):
                os.makedirs(des_path)

            for i in range(*rang, interval):
                ind = str(i).zfill(6)

                for ext in exts:
                    shutil.move(os.path.join(src_dir, scene_dir, rsu_dir, ind+ext), os.path.join(des_dir, scene_dir, rsu_dir, ind+ext))

if __name__ == '__main__':

    main()