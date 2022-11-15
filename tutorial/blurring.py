import numpy as np
import cv2 as cv

# Funcion de callback para switch
def updateBlurring(_):
    x = cv.getTrackbarPos('Tamano', 'image')
    blurring(x)

# Funcion de callback
def blurring(x):
    global img, switch
    s = cv.getTrackbarPos(switch, 'image')
    if x == 0:
        blur = img
    else:
        if s == 0:
            blur = cv.blur(img,(x,x))
        elif s == 1:
            x = int(int(x / 2) * 2 + 1)
            cv.setTrackbarPos('Tamano', 'image', x)
            blur = cv.GaussianBlur(img,(x,x), 0)
        else:
            x = int(int(x / 2) * 2 + 1)
            cv.setTrackbarPos('Tamano', 'image', x)
            blur = cv.medianBlur(img, x)
    cv.imshow('image', blur)

img = cv.imread('images/duckietown.png', 1)

# Crea ventana de nombre 'image'
cv.namedWindow('image')

# Crea trackbars para manejar el tamano del filtro
cv.createTrackbar('Tamano', 'image', 1, 50, blurring)

# Crea switch para elegir el tipo de filtro
switch = '0: Average \n1: Gaussian \n 2: Median\n'
cv.createTrackbar(switch, 'image', 0, 2, updateBlurring)

cv.imshow('image', img)

while(True):
    k = cv.waitKey(1)
    # Tecla 'Esc' cierra el programa
    if k == 27:
        break
cv.destroyAllWindows()
