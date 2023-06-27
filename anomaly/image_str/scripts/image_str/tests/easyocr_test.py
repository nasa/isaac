import os

import cv2
import easyocr

reader = easyocr.Reader(
    ["en"]
)  # this needs to run only once to load the model into memory

# borrowed from https://github.com/lengstrom/fast-style-transfer/blob/master/src/utils.py
def get_files(img_dir):
    imgs, masks, xmls = list_files(img_dir)
    return imgs, masks, xmls


def list_files(in_path):
    img_files = []
    mask_files = []
    gt_files = []
    for (dirpath, dirnames, filenames) in os.walk(in_path):
        for file in filenames:
            filename, ext = os.path.splitext(file)
            ext = str.lower(ext)
            if (
                ext == ".jpg"
                or ext == ".jpeg"
                or ext == ".gif"
                or ext == ".png"
                or ext == ".pgm"
            ):
                img_files.append(os.path.join(dirpath, file))
            elif ext == ".bmp":
                mask_files.append(os.path.join(dirpath, file))
            elif ext == ".xml" or ext == ".gt" or ext == ".txt":
                gt_files.append(os.path.join(dirpath, file))
            elif ext == ".zip":
                continue
    # img_files.sort()
    # mask_files.sort()
    # gt_files.sort()
    return img_files, mask_files, gt_files


test_folder = "../images/"
result_folder = "../result/trade_study/easyocr/"

image_list, _, _ = get_files(test_folder)

for k, image_path in enumerate(image_list):
    print(
        "Test image {:d}/{:d}: {:s}\n".format(k + 1, len(image_list), image_path),
        end="\r",
    )
    image = cv2.imread(image_path)  # read image
    result = reader.readtext(image)
    filename, file_ext = os.path.splitext(os.path.basename(image_path))

    for res in result:
        top_left = (int(res[0][0][0]), int(res[0][0][1]))  # convert float to int
        bottom_right = (int(res[0][2][0]), int(res[0][2][1]))  # convert float to int
        image = cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 3)
        image = cv2.putText(
            image,
            res[1],
            (top_left[0], top_left[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 0, 255),
            2,
        )
    print(result_folder + image_path)
    cv2.imwrite(result_folder + filename + ".jpg", image)
