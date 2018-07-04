// Copyright (c) 2012, 2013
// Ravi Peters -- r.y.peters@tudelft.nl
// All rights reserved
// 
// This file is part of Surfonoi.
// 
// Surfonoi is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// Surfonoi is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Surfonoi.  If not, see <http://www.gnu.org/licenses/>.

#include "Raster.h"

Raster::Raster(const char* WKGCS, double cellsize, double min_x, double max_x, double min_y, double max_y):
        cellSize(cellsize), minx(min_x), maxx(max_x), miny(min_y), maxy(max_y)
{
    if( EQUALN(WKGCS, "EPSG:",5) ) {
        oSRS.importFromEPSG( atoi(WKGCS+5) );
    } else if (EQUALN(WKGCS, "EPSGA:",6)) {
        oSRS.importFromEPSGA( atoi(WKGCS+6) );
    }
    
    dimx = (maxx-minx)/cellSize + 1;
    dimy = (maxy-miny)/cellSize + 1;
    GDALAllRegister();
}

void Raster::prefill_arrays(double * vals, int16_t * counts, alg a){
  if (a==MIN)
      noDataVal = 99999;
  else if (a==CNT)
      noDataVal = 0;
  else
      noDataVal = -99999;
    
  std::fill(vals+0, vals+dimx*dimy, noDataVal);
  std::fill(counts+0, counts+dimx*dimy, 0);
}

void Raster::add_point(double x, double y, double z, double vals[], int16_t counts[], alg a)
{
    if (a==MIN) {
        min(x,y,z, vals);
    } else if (a==MAX) {
        max(x,y,z, vals);
    } else if (a==AVG) {
        avg(x,y,z, vals, counts);
    } else if (a==CNT) {
        cnt(x,y, counts);
    }
}

inline void Raster::avg(double &x, double &y, double &val, double vals[], int16_t cnt[])
{
    size_t c = getCoord(x,y);
    vals[c]= (vals[c]*cnt[c]+val)/(cnt[c]+1);
    ++cnt[c];
}

inline void Raster::min(double &x, double &y, double &val, double vals[])
{
    size_t c = getCoord(x,y);
    if (vals[c]>val) vals[c] = val;
}

inline void Raster::max(double &x, double &y, double &val, double vals[])
{
    size_t c = getCoord(x,y);
    if (vals[c]<val) vals[c] = val;
}

inline void Raster::cnt(double &x, double &y, int16_t cnt[])
{
    size_t c = getCoord(x,y);
    ++cnt[c];
}

size_t Raster::getCoord(double &x, double &y)
{
    size_t r = static_cast<size_t>( floor((y-miny) / cellSize) );
    size_t c = static_cast<size_t>( floor((x-minx) / cellSize) );
    
    return r * dimx + c;
}

void Raster::write(alg a, void * dataPtr, const char* outFile)
{
    GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName("GTiff");
    GDALDataset *poDstDS;
    GDALDataType dataType;

    if (a == CNT)
        dataType = GDT_UInt16;
    else
        dataType = GDT_Float64;
    
    char **papszOptions = NULL;
    poDstDS = poDriver->Create( outFile, dimx, dimy, 1, dataType,
                               papszOptions );
    double adfGeoTransform[6] = { minx, cellSize, 0, miny, 0, cellSize };
    GDALRasterBand *poBand;
    
    poDstDS->SetGeoTransform( adfGeoTransform );
    
    //    std::cout << oSRS.SetWellKnownGeogCS( WKGCS );
    //    std::cout << pszSRS_WKT <<std::endl;
    
    char *pszSRS_WKT = NULL;
    oSRS.exportToWkt( &pszSRS_WKT );
    poDstDS->SetProjection( pszSRS_WKT );
    CPLFree( pszSRS_WKT );
    
    poBand = poDstDS->GetRasterBand(1);
    poBand->RasterIO( GF_Write, 0, 0, dimx, dimy,
                     dataPtr, dimx, dimy, dataType, 0, 0 );
    poBand->SetNoDataValue(noDataVal);
    /* Once we're done, close properly the dataset */
    GDALClose( (GDALDatasetH) poDstDS );
}