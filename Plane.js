var VectorTools = require("./VectorTools.js");

/*
  simple verification from casio website: http://bit.ly/29LMfQ9
*/
var Plane = function(){
  // the plane equation is:
  // ax + by + cz + d = 0
  // here are those coeficents:
  this._a = null;
  this._b = null;
  this._c = null;
  this._d = null;

  this._n = null;   // the normal vector to the plane
  this._u = null;   // one of the vector that belong to the plane
  this._v = null;   // another vector that belong to the plane, orthogonal to _u

  this._vecTools = new VectorTools();    // some (simple) tool to perform vector calculus
}


/*
  initialize the equation of the plane from 3 points
  each point is a Array [x, y, z]
*/
Plane.prototype.makeFromThreePoints = function(P, Q, R){
  // vector from P to Q
  var vPQ = [Q[0] - P[0], Q[1] - P[1], Q[2] - P[2]];
  var vPR = [R[0] - P[0], R[1] - P[1], R[2] - P[2]];

  this._n = this._vecTools.crossProduct(vPQ, vPR, false);

  this._a = this._n[0];
  this._b = this._n[1];
  this._c = this._n[2];
  this._d = (-1) * (this._a * P[0] + this._b * P[1] + this._c * P[2] );

  this._defineUandVwith2points(P, Q);

}



/*
  initialize the equation of the plane from 1 point and a vector.
  The point does not have to be within the cube.
  point and vector are both tuple (x, y, z)
*/
Plane.prototype.makeFromOnePointAndNormalVector = function(point, vector){
    this._n = vector;
    this._a = this._n[0];
    this._b = this._n[1];
    this._c = this._n[2];
    this._d = (-1) * (this._a * point[0] + this._b * point[1] + this._c * point[2] );

    // find another point on the plane...

    // The next 3 cases are for when a plane is (at least in 1D)
    // aligned with the referential
    var point2 = null;

    // case 1
    if(this._c != 0){
      var x2 = point[0] + 1;
      var y2 = point[1];
      var z2 = (-1) * ( (this._a * x2 + this._b * y2 + this._d) / this._c );
      point2 = (x2, y2, z2);
    }

    // case 2
    if(this._b != 0 && !point2){
      var x2 = point[0] + 1;
      var z2 = point[2];
      var y2 = (-1) * ( (this._a * x2 + this._c * z2 + this._d) / this._b );
      point2 = (x2, y2, z2);
    }

    // case 3
    if(this._a != 0 && !point2){
      var y2 = point[1] + 1;
      var z2 = point[2];
      var x2 =  (-1) * ( (this._b * y2 + this._c * z2 + this._d) / this._a );
      point2 = (x2, y2, z2);
    }

    this._defineUandVwith2points(point, point2);
}


/*
  return the abcd factors of the plane equation as a tuple
  assuming ax + by + cz + d = 0
*/
Plane.prototype.getPlaneEquation = function(){
  return [this._a, this._b, this._c, this._d];
}


/*
  return tuple with normal the vector
*/
Plane.prototype.getNormalVector = function(){
  return this._n;
}


/*
  u and v are two vectors frome this plane.
  they are orthogonal and normalize so that we can build a regular grid
  along this plane.
  BEWARE: the equation and normal to the plane must be set
  Some calculus hints come from there: http://bit.ly/29coWgs .
  args: P and Q are two points from the plane. vPQ, when normalized
  will be used as u
*/
Plane.prototype._defineUandVwith2points = function(P, Q){
  this._u = this._vecTools.normalize([
      Q[0] - P[0],
      Q[1] - P[1],
      Q[2] - P[2]
    ]);

  this._v = this._vecTools.crossProduct(this._u, this._n);
}


/*
  return the unit vector u as a tuple (x, y, z)
*/
Plane.prototype.getUvector = function(){
  return this._u;
}


/*
  return the unit vector v as a tuple (x, y, z)
*/
Plane.prototype.getVvector = function(){
  return this._v;
}


function testPlane(){
  var p = new Plane();
  //p.makeFromThreePoints( [1, -2, 0], [3, 1, 4], [0, -1, 2]);
  //p.makeFromThreePoints( [1, -2, 0], [3, 10, 4], [0, -1, 2])
  p.makeFromThreePoints( [0, 0, 0], [1, 0, 0], [0, 1, 1]);
  console.log(p.getPlaneEquation() );
}

//testPlane();
