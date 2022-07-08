from tokenize import String
import requests, tarfile, os

def downloadFile(url:String, target_path:String):
    response = requests.get(url, stream=True)
    if response.status_code == 200:
        with open(target_path, 'wb') as f:
            f.write(response.raw.read())


def extractTarFile(target_path:String):
    file = tarfile.open(target_path)
    file.extractall('./output') # creates folder labeled "output" if it doesn't already exists
    file.close()


def deleteFile(file:String):
    '''Just deletes the tar file that is unnecessary'''
    print("[INFO] Cleaning up unnecessary files") 
    if os.path.exists(file):
        os.remove(file)
    else:
        print("File does not exists")


def main():
    # if you are trying to install a different default model change the url and target_path
    url = 'http://download.tensorflow.org/models/object_detection/tf2/20200711/centernet_resnet101_v1_fpn_512x512_coco17_tpu-8.tar.gz'
    target_path = 'CenterNet Resnet101 V1 FPN 512x512'

    condition = input("[INFO] Default installation will be Faster-RCNN Inception v2 \nWould you like to choose model? [y/n]: ")
    if condition.lower() == 'n':
        print("[INFO] Depending your wifi speed this may take a moment")
        downloadFile(url=url, target_path=target_path)
        extractTarFile(target_path)
        deleteFile(target_path)
    else: 
        # Currently does not work. Wait until Jason wakes up from his 12 hour nap
        url = input("URL of model: ")
        target_path = input("Name of pre-trained model: ")

        print("[INFO] Depending your wifi speed this may take a moment")
        downloadFile(url=url, target_path=target_path)
        extractTarFile(target_path)
        deleteFile(target_path)


main()
print("(`-`)> file downloaded and converted (`-`)>")
