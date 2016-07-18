/*

*/

var ObliqueSampler = function(volume, plane){
  this._3Ddata = volume;
  this._plane = plane;

  this._planePolygon = null; // the polygon formed by the intersection of the plane and the cube of data (from 3 to 6 vertice)
  this._samplingFactor = 1.; // impact the size of the finale image

  this._vecTools = new VectorTools();

  // contains the oblique image (as a 1D typed array), the equivalent mask and few other info.
  // created by initObliqueImage
  this._obliqueImage = null;

}

/*
  important in case the plane changes (rotation or translation).
  for now, it's only about the intersection polygon, but other method
  may add up later
*/
ObliqueSampler.prototype.update = function(){
  this.computeCubePlaneHitPoints();
}


/*
  takes all the edges or the cube (12 in total)
  and make the intersection with the plane.
  For one single edge, there can be:
  - no hit (the edge doesnt cross the plane)
  - one hit (the edge crosses the plane)
  - an infinity of hits (the edge belongs to the plane)
*/
ObliqueSampler.prototype.computeCubePlaneHitPoints = function(){
  var cubeEdges = this._3Ddata.getEdgesEquations();
  var hitPoints = [];

  for(var i=0; i<cubeEdges.length; i++){
    var edge = cubeEdges[i];
    var tempHitPoint = this._getHitPoint(edge[0], edge[1], this._plane);

    // 1- We dont want to add infinite because it mean an orthogonal edge
    // from this one (still of the cube) will cross the plane in a single
    // point -- and this later case is easier to deal with.
    // 2- Check if hitpoint is within the cube.
    // 3- Avoid multiple occurence for the same hit point
    if( tempHitPoint && // may be null if contains Infinity as x, y or z
        this._3Ddata.isWithin(tempHitPoint, true))
    {
        var isAlreadyIn = false;

        // check if the point is already in the array
        for(var hp=0; hp<hitPoints.length; hp++ ){
          if( hitPoints[hp][0] == tempHitPoint[0] &&
              hitPoints[hp][1] == tempHitPoint[1] &&
              hitPoints[hp][2] == tempHitPoint[2]){
                isAlreadyIn = true;
                break;
              }
        }

        if(!isAlreadyIn){
          hitPoints.push(tempHitPoint);
        }
    }

  }

  // array are still easier to deal with
  this._planePolygon = hitPoints;
}



/*
  return a point in 3D space (tuple (x, y, z) ).
  vector and point define a "fixed vector" (droite affine)
  both are tuple (x, y, z)
  plane is the plane equation as a tuple (a, b, c, d)
*/
ObliqueSampler.prototype._getHitPoint = function(vector, point, plane){

  // 3D affine system tuple:
  // ( (l, alpha), (m, beta), (n, gamma) )
  var affineSystem = this._vecTools.affine3DFromVectorAndPoint(vector, point);

  // equation plane as ax + by + cz + d = 0
  // this tuple is (a, b, c, d)
  var planeEquation = plane.getPlaneEquation();

  // system resolution for t:
  // t = (a*l + b*m + c*n + d) / ( -1 * (a*alpha + b*beta + c*gamma) )

  var tNumerator = ( planeEquation[0]* affineSystem[0][0] +
        planeEquation[1]* affineSystem[1][0] +
        planeEquation[2]* affineSystem[2][0] +
        planeEquation[3] );

  var tDenominator = (-1) *
      ( planeEquation[0]* affineSystem[0][1] +
        planeEquation[1]* affineSystem[1][1] +
        planeEquation[2]* affineSystem[2][1] );

  // TODO: be sure the cast to float is done
  // float conversion is mandatory to avoid euclidean div...
  //var t = float(tNumerator) / float(tDenominator);
  var t = tNumerator / tDenominator;

  // injection of t to the 3D affine system:
  var x =  affineSystem[0][0] + affineSystem[0][1] * t;
  var y =  affineSystem[1][0] + affineSystem[1][1] * t;
  var z =  affineSystem[2][0] + affineSystem[2][1] * t;


  // dont bother returning a vector containing Infinity, just return null.
  // (it will be spotted)
  if( x == Infinity ||
      y == Infinity ||
      z == Infinity)
  {
    return null;
  }

  // otherwise, return the 3D point
  return [x, y, z]
}


/*
  return the center of the polygon
  as a tuple (x, y, z)
*/
ObliqueSampler.prototype._getStartingSeed = function(){
  if(this._planePolygon){

    var xSum = 0;
    var ySum = 0;
    var zSum = 0;
    var numOfVertex = this._planePolygon.length

    for(var v=0; v<numOfVertex; v++){
      xSum += this._planePolygon[v][0];
      ySum += this._planePolygon[v][1];
      zSum += this._planePolygon[v][2];
    }

    // TODO: be sure it is casted to float
    /*
    var xCenter = float(xSum) / numOfVertex;
    var yCenter = float(ySum) / numOfVertex;
    var zCenter = float(zSum) / numOfVertex;
    */
    var xCenter = xSum / numOfVertex;
    var yCenter = ySum / numOfVertex;
    var zCenter = zSum / numOfVertex;

    return [xCenter, yCenter, zCenter];

  }else{
    console.log("ERROR: the polygon is not defined yet, you should call the update() method");
    return null;
  }

}




/*
  return the diagonal (length) of the polygon bounding box.
  affected by the _samplingFactor (ie. doubled if 2)
*/
ObliqueSampler.prototype._getLargestSide = function(){

    if(this._planePolygon){

        var xMin = this._planePolygon[0][0];
        var yMin = this._planePolygon[0][1];
        var zMin = this._planePolygon[0][2];

        var xMax = this._planePolygon[0][0];
        var yMax = this._planePolygon[0][1];
        var zMax = this._planePolygon[0][2];

        for(var v=0; v<this._planePolygon.length; v++){
          var vertex = this._planePolygon[v];

          xMin = Math.min(xMin, vertex[0]);
          xMax = Math.max(xMax, vertex[0]);

          yMin = Math.min(yMin, vertex[1]);
          yMax = Math.max(yMax, vertex[1]);

          zMin = Math.min(zMin, vertex[2]);
          zMax = Math.max(zMax, vertex[2]);
        }



        var boxSide = Math.sqrt((xMax-xMin)*(xMax-xMin) + (yMax-yMin)*(yMax-yMin) + (zMax-zMin)*(zMax-zMin));

        return boxSide * this._samplingFactor;

    }else{
      console.log("ERROR: the polygon is not defined yet, you should call the update() method");
      return null;
    }
}


/*
  we always start to fill the oblique image from its 2D center (in arg)
  the center of the 2D oblique image matches the 3D starting seed
  (center of the inner polygon, made by the intersection of the
  plane with the cube)
*/
ObliqueSampler.prototype.obliqueImageCoordToCubeCoord = function(centerImage, startingSeed, dx, dy){
  var u = this._plane.getUvector(); // u goes to x direction (arbitrary)
  var v = this._plane.getVvector(); // v goes to y direction (arbitrary)

  var target3Dpoint = [
    startingSeed[0] + dx * u[0] / this._samplingFactor + dy * v[0] / this._samplingFactor,
    startingSeed[1] + dx * u[1] / this._samplingFactor + dy * v[1] / this._samplingFactor,
    startingSeed[2] + dx * u[2] / this._samplingFactor + dy * v[2] / this._samplingFactor
  ];

  return target3Dpoint;
}



/*
  returns True if the 3D coord matching to this oblique image point
  is within the cube.
  Returns False if outside the cube.
*/
ObliqueSampler.prototype.isImageCoordInCube = function(centerImage, startingSeed, dx, dy){
  var cubeCoord = this.obliqueImageCoordToCubeCoord(centerImage, startingSeed, dx, dy);
  return this._3Ddata.isWithin(cubeCoord, false);
}


/*
  Define the sampling factor (default = 1).
  Has to be called before startSampling()
*/
ObliqueSampler.prototype.setSamplingFactor = function(f){
  this._samplingFactor = f;
}


/*
  initialize the oblique image array  as a 1D array.
  Width and height are also packaged in the structure.
*/
ObliqueSampler.prototype.initObliqueImage = function(datatype, width, height){

  var imageTypedArray = null;

  // we could simply use image.type, but written types are easier to read...
  switch (datatype) {
    case 'int8':
      imageTypedArray = new Int8Array(width * height);
      break;
    case 'int16':
      imageTypedArray = new Int16Array(width * height);
      break;
    case 'int32':
      imageTypedArray = new Int32Array(width * height);
      break;
    case 'float32':
      imageTypedArray = new Float32Array(width * height);
      break;
    case 'float64':
      imageTypedArray = new Float64Array(width * height);
      break;
    case 'uint8':
      imageTypedArray = new Uint8Array(width * height);
      break;
    case 'uint16':
      imageTypedArray = new Uint16Array(width * height);
      break;
    case 'uint32':
      imageTypedArray = new Uint32Array(width * height);
      break;
    default:
      var error_message = "Unsupported data type: " + header.datatype;
      console.log({ message: error_message } );
  }

  this._obliqueImage = {
    data: imageTypedArray,
    maskData: new Int8Array(width * height),
    width: width,
    height: height
  };

}


/*
  get the pixel value of the oblique image at (x, y)
*/
ObliqueSampler.prototype.getImageValue = function(x, y){
  if(x >= 0 && y>=0 && x<this._obliqueImage.width && y<this._obliqueImage.height){
    return this._obliqueImage.data[x*this._obliqueImage.width + y];
  }else{
    console.log("ERROR: (" + x + " , " + y + ") is out of the image");
    return undefined;
  }
}


/*
  set the pixel value of the oblique image at (x, y)
*/
ObliqueSampler.prototype.setImageValue = function(x, y, value){
  if(x >= 0 && y>=0 && x<this._obliqueImage.width && y<this._obliqueImage.height){
    this._obliqueImage.data[x*this._obliqueImage.width + y] = value;
  }else{
    console.log("ERROR: (" + x + " , " + y + ") is out of the image");
  }
}


/*
  get the pixel value of the oblique mask at (x, y).
  returns -1 if out of image (easier to catch)
*/
ObliqueSampler.prototype.getMaskValue = function(x, y){
  if(x >= 0 && y>=0 && x<this._obliqueImage.width && y<this._obliqueImage.height){
    return this._obliqueImage.maskData[x*this._obliqueImage.width + y];
  }else{
    console.log("ERROR: (" + x + " , " + y + ") is out of the image/mask");
    return -1;
  }
}


/*
  set the pixel value of the oblique mask at (x, y)
*/
ObliqueSampler.prototype.setMaskValue = function(x, y, value){
  if(x >= 0 && y>=0 && x<this._obliqueImage.width && y<this._obliqueImage.height){
    this._obliqueImage.maskData[x*this._obliqueImage.width + y] = value;
  }else{
    console.log("ERROR: (" + x + " , " + y + ") is out of the image/mask");
  }
}


/*
  export canvas compatible data - nice for display
*/
ObliqueSampler.prototype.exportForCanvas = function( factor){



  if(!this._obliqueImage){
    console.log("ERROR: the oblique image is empty.");
    return null;
  }

  factor = factor || 255;

  var simpleImageContext = document.createElement("canvas").getContext("2d");
  var width = this._obliqueImage.width;
  var height = this._obliqueImage.height;

  var image = simpleImageContext.createImageData(width, height);
  var rgbaArray = new Uint8ClampedArray(this._obliqueImage.data.length * 4);

  // from single channel array to RGBA buff, just repeat the value...
  for(var i=0; i<this._obliqueImage.data.length; i++){
    rgbaArray[i*4] = this._obliqueImage.data[i] * factor;
    rgbaArray[i*4 +1] = this._obliqueImage.data[i] * factor;
    rgbaArray[i*4 +2] = this._obliqueImage.data[i] * factor;
    rgbaArray[i*4 +3] = 255;
  }

  image.data.set(rgbaArray, 0);

  return image;
}

/*
  start the sampling/filling process.
  interpolate:
    False (default) = nearest neighbor, crispier
    True = trilinear (3D) interpolation, slower, smoother
*/
ObliqueSampler.prototype.startSampling = function(filepath, interpolate){

  var dataType = this._3Ddata.getDataType();
  var largestSide = Math.ceil(this._getLargestSide());
  var startingSeed = this._getStartingSeed();

  var obliqueImageCenter = [ Math.round(largestSide / 2), Math.round(largestSide / 2) ];

  // will contain the (interpolated) data from the cube
  //var obliqueImage = np.zeros((int(largestSide), int(largestSide)), dtype=dataType )
  // initialize this._obliqueImage and the mask
  this.initObliqueImage(this._3Ddata.getDataType(), largestSide, largestSide);


  // mask of boolean to track where the filling algorithm has already been
  //var obliqueImageMask = np.zeros((int(largestSide), int(largestSide)), dtype=dataType  )

  // stack used for the fillin algorithm
  var pixelStack = [];
  pixelStack.push(obliqueImageCenter);

  var counter = 0;

  console.log("start sampling...");

  while(pixelStack.length > 0){
    var currentPixel = pixelStack.pop();
    var x = currentPixel[0];
    var y = currentPixel[1];

    // if the image was not filled here...
    if(this.getMaskValue(x, y) == 0){
      // marking the mask (been here!)
      this.setMaskValue(x, y, 255);

      var cubeCoord = this.obliqueImageCoordToCubeCoord(obliqueImageCenter, startingSeed, x - obliqueImageCenter[0], y - obliqueImageCenter[1]);

      // get the interpolated color of the currentPixel from 3D cube
      //var color = this._3Ddata.getValueTuple(cubeCoord, interpolate);
      var color = this._3Ddata.getIntensityValue(cubeCoord[0], cubeCoord[1], cubeCoord[2]);


      // painting the image
      if(color){
          this.setImageValue(x, y, color);
      }

      // going north
      var yNorth = y + 1;
      var xNorth = x;
      if(this.getMaskValue(xNorth, yNorth) == 0){
        if(this.isImageCoordInCube(obliqueImageCenter, startingSeed, xNorth - obliqueImageCenter[0], yNorth - obliqueImageCenter[1]) ){
            pixelStack.push([xNorth, yNorth]);
        }
      }

      // going south
      var ySouth = y - 1;
      var xSouth = x;
      if(this.getMaskValue(xSouth, ySouth) == 0){
        if(this.isImageCoordInCube(obliqueImageCenter, startingSeed, xSouth - obliqueImageCenter[0], ySouth - obliqueImageCenter[1])){
            pixelStack.push([xSouth, ySouth]);
        }
      }

      // going east
      var yEast = y;
      var xEast = x + 1;
      if(this.getMaskValue(xEast, yEast) == 0){
        if(this.isImageCoordInCube(obliqueImageCenter, startingSeed, xEast - obliqueImageCenter[0], yEast - obliqueImageCenter[1])){
            pixelStack.push( [xEast, yEast] );
        }
      }

      // going west
      var yWest = y
      var xWest = x - 1
      if(this.getMaskValue(xWest, yWest) == 0){

        if(this.isImageCoordInCube(obliqueImageCenter, startingSeed, xWest - obliqueImageCenter[0], yWest - obliqueImageCenter[1])){

            pixelStack.push( [xWest, yWest] );
          }
      }




    }


    if(counter%100 == 0){
      console.log(counter);
    }
    counter ++;


  }
}
