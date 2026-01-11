import cv2

def rescaleFrame(frame,scale=1):
    width = int(frame.shape[1] * scale) #ancho
    height = int(frame.shape[0] * scale) #alto
    dimensions = (width,height)          #tupla que contiene las dimensiones reales
    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)
