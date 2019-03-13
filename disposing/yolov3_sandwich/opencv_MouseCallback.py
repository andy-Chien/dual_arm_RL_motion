import numpy as np
import cv2, os, sys, getopt
import argparse

parser = argparse.ArgumentParser()

parser.add_argument('--TrainDIR', type=str, default='.', help='path to Trainingdata')
parser.add_argument('--SaveDIR', type=str, default='.', help='path to train.txt')
parser.add_argument('--Label', type=str, default='.', help='Number of object label')
FLAGS = parser.parse_args()

print("FLAGS.TrainDIR: ", FLAGS.TrainDIR)

# Global Variables
helpMessage = 'Usage:\n\tcrop.py <command> [argument]\n\nCommands:\n\t-h --help\t\tDisplay this help message\n\t-i --image [path]\tInput image\n\t-f --folder [path]\tImage Folder\n\t-r --regex [regex]\tNaming of output files [WIP]\n\t-s --save [path]\tPath to Save'

# Arguments
pathIMG = ''
pathDIR = ''
pathSAV = ''
regex = ''

pathDIR = FLAGS.TrainDIR
pathSAV = FLAGS.SaveDIR
object_label = FLAGS.Label
# Graphical
drawing = False # true if mouse is pressed
cropped = False
ix,iy = -1,-1
rx,ry = -1, -1
# MISC
img_index = 0
# Mouse callback function
def draw(event,x,y,flags,param):
    global ix, iy, rx, ry, drawing, img, DEFAULT, cropped , pathIMG

    cropped = False

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            img = DEFAULT.copy()
            cv2.imshow(pathIMG, img)
            if (abs(ix - x) < abs(iy -y)):
                cv2.rectangle(img,(ix,iy),(x,y),(0,255,0),5)
                rx, ry = x, y
            else:
                cv2.rectangle(img,(ix,iy),(x,y),(0,255,0),5)
                rx, ry = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        #crop(ix,iy,x,y)

# Get Arguments
def getArgvs(argv):
    try:
        opts, args = getopt.getopt(argv,"hi:f:r:s:",["help","image=","folder=","regex=","save="])
    except getopt.GetoptError:
        print(helpMessage)
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', "--help"):
            print(helpMessage)
            sys.exit()
        elif opt in ("-i", "--image"):
            global pathIMG
            pathIMG = arg
            #print("img: " + arg)
        elif opt in ("-f", "--folder"):
            global pathDIR
            pathDIR = arg
            #print("dir: " + arg)
        elif opt in ("-r","--regex"):
            global regex
            regex = arg
            #print("regex: " + arg)
        elif opt in ("-s","--save"):
            global pathSAV
            pathSAV = arg

# Crop Image
def crop(ix,iy,x,y):
    global img, DEFAULT, cropped

    img = DEFAULT.copy()
    cv2.imshow(pathIMG,img)

    if (abs(ix - x) < abs(iy -y)):
        img = img[iy:y, ix:x]
    else:
        img = img[iy:y, ix:x]

# Save image
def save(crop_img):
    # Set name
    name = pathSAV + "img" + str(img_index) + ".png"

    # Resize
    dst = cv2.resize(crop_img, (32,32))

    # Save
    cv2.imwrite(name,dst)
    print("Saved image sucsessfully")

def WriteRectangle():
    global ix, iy, rx, ry, pathIMG
    
    line = str(pathIMG) +" " + str(ix) + ',' + str(iy) + ',' + str(rx) + ',' + str(ry) + ',' + str(object_label) + '\n'
    with open(pathSAV+'train.txt', 'a') as f:
        f.writelines(line)

# Main Loop
def loop():
    global img, pathIMG

    cv2.namedWindow(pathIMG,0)
    cv2.setMouseCallback(pathIMG, draw)

    while (1):
        cv2.imshow(pathIMG, img)
        k = cv2.waitKey(1) & 0xFF
        if (k == 27):
            print("Cancelled Crop")
            break
        elif (k == ord('s')):# and cropped):
            WriteRectangle()
            print("Image Saved")
            #save(img)
            break

    print("Done!")
    cv2.destroyAllWindows()

# Iterate through images in path
def getIMG(path):
    global img, DEFAULT, pathIMG
    directory = os.fsencode(path)
    for filename in os.listdir(directory):
        # Get Image Path
        pathIMG = path + filename.decode("utf-8")
        print("pathIMG:", pathIMG)

        # Read Image
        img = cv2.imread(pathIMG,-1)
        
        DEFAULT = img.copy()

        # Draw image
        loop()

    return 0

# Main Function
def main():
    #getArgvs(sys.argv[1:])
    global img, DEFAULT

    if (pathDIR != ''):
        # Print Path
        print("pathDIR: " + pathDIR)

        # Cycle through files
        getIMG(pathDIR)

    elif (pathIMG != ''):
        # Print Path
        print("img: " + pathIMG)

        # Load Image
        img = cv2.imread(pathIMG,-1)
        DEFAULT = img.copy()

        # Draw Image
        loop()

# Run Main
if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()
