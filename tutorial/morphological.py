import numpy as np
import cv2 as cv

# Funcion de callback para switch
def updateMorpho(_):
    x = cv.getTrackbarPos('Tamaño', 'image')
    morphological(x)

# Funcion de callback
def thresholding(x):
    global img, thresh
    _, thresh = cv.threshold(img, x, 255, cv.THRESH_BINARY)
    updateMorpho(0)

# Funcion de callback
def morphological(x):
    kernelTypes = [cv.MORPH_RECT, cv.MORPH_ELLIPSE, cv.MORPH_CROSS]
    global thresh, switch, k_switch
    x = int(int(x / 2) * 2 + 1)
    cv.setTrackbarPos('Tamaño', 'image', x)
    idx = cv.getTrackbarPos(k_switch, 'image')
    kernel = cv.getStructuringElement(kernelTypes[idx], (x, x))

    s = cv.getTrackbarPos(switch, 'image')
    if s == 0:
        morpho = cv.erode(thresh, kernel, iterations = 1)
    else:
        morpho = cv.dilate(thresh, kernel, iterations = 1)
    cv.imshow('image', morpho)

img = cv.imread('images/pato.jpg', 0)
_, thresh = cv.threshold(img, 127, 255, cv.THRESH_BINARY)
# Crea ventana de nombre 'image'
cv.namedWindow('image')

# Crea trackbars para manejar el tamano del filtro
cv.createTrackbar('Tamaño', 'image', 1, 50, morphological)

# Crea switch para elegir la operacion morfologica
switch = '0: Erode \n1: Dilate\n'
cv.createTrackbar(switch, 'image', 0, 1, updateMorpho)

# Crea switch para elegir el tipo de kernel
k_switch = '0: Rect \n1: Ellipse\n2: Cross\n'
cv.createTrackbar(k_switch, 'image', 0, 2, updateMorpho)

# Define umbral de binarizacion
cv.createTrackbar('Umbral', 'image', 127, 255, thresholding)

cv.imshow('image', thresh)

while(True):
    k = cv.waitKey(1)
    # Tecla 'Esc' cierra el programa
    if k == 27:
        break
cv.destroyAllWindows()
