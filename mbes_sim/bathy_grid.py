from osgeo import gdal

class BathyGrid:
    def __init__(self, fname):
        gdal.UseExceptions()
        self.dataset = gdal.Open(fname, gdal.GA_ReadOnly)
        self.band = self.dataset.GetRasterBand(1)
        self.geoTransform = self.dataset.GetGeoTransform()
        self.inverseGeoTransform = gdal.InvGeoTransform(self.geoTransform)
        self.data = self.band.ReadAsArray()
        
        sourceSR = gdal.osr.SpatialReference()
        sourceSR.SetWellKnownGeogCS("WGS84")
        
        targetSR = gdal.osr.SpatialReference()
        targetSR.ImportFromWkt(self.dataset.GetProjection())
        
        self.coordinateTransformation = gdal.osr.CoordinateTransformation(sourceSR, targetSR)

    def getXY(self,lat,lon):
        return self.coordinateTransformation.TransformPoint(lat,lon)[:2]
        
    def getDepthAtLatLon(self,lat,lon):
        x,y = self.getXY(lat,lon)
        return self.getDepth(x,y)
    
    def getDepth(self,x,y):
        xi = self.inverseGeoTransform[0]+x*self.inverseGeoTransform[1]+y*self.inverseGeoTransform[2]
        yi = self.inverseGeoTransform[3]+x*self.inverseGeoTransform[4]+y*self.inverseGeoTransform[5]
        try:
            return self.data[int(yi),int(xi)]
        except IndexError:
            return None
