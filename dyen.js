/*
 Copyright (c) 2006, 2007 Alec Cove
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of this
 software and associated documentation files (the "Software"), to deal in the Software
 without restriction, including without limitation the rights to use, copy, modify,
 merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to the following
 conditions:
 
 The above copyright notice and this permission notice shall be included in all copies
 or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
 
/*
 ported by flashrod 2008
 */
/** 2D vector
 */
var Vector = Class.create({
  initialize: function(x, y) {
    this.x = x || 0;
    this.y = y || 0;
  },
  setTo: function(x, y) {
    this.x = x;
    this.y = y;
  },
  copy: function(v) {
    this.x = v.x;
    this.y = v.y;
  },
  dot: function(v) {
    return this.x * v.x + this.y * v.y;
  },
  cross: function(v) {
    return this.x * v.y - this.y * v.x;
  },
  plus: function(v) {
    return new Vector(this.x + v.x, this.y + v.y);
  },
  plusEquals: function(v) {
    this.x += v.x;
    this.y += v.y;
    return this;
  },
  add: function(v) {
    return new Vector(this.x + v.x, this.y + v.y);
  },
  addEquals: function(v) {
    this.x += v.x;
    this.y += v.y;
    return this;
  },
  minus: function(v) {
    return new Vector(this.x - v.x, this.y - v.y);
  },
  minusEquals: function(v) {
    this.x -= v.x;
    this.y -= v.y;
    return this;
  },
  mult: function(s) {
    return new Vector(this.x * s, this.y * s);
  },
  multEquals: function(s) {
    this.x *= s;
    this.y *= s;
    return this;
  },
  times: function(v) {
    return new Vector(this.x * v.x, this.y * v.y);
  },
  divEquals: function(s) {
    if (s == 0) {
      s = 0.0001;
    }
    this.x /= s;
    this.y /= s;
    return this;
  },
  magnitude: function() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  },
  distance: function(v) {
    var delta = this.minus(v);
    return delta.magnitude();
  },
  normalize: function() {
    var m = this.magnitude();
    if (m == 0) {
      m = 0.0001;
    }
    return this.mult(1 / m);
  },
  toString: function() {
    return "(" + this.x + ", " + this.y + ")";
  }
});
var Particle = Class.create({
    initialize: function(x, y, fixed, mass, elasticity, friction) {
        this.samp = new Vector(0, 0);
        this.interval = { min:0, max:0 };
        this.forces = new Vector(0, 0);
        this.collision = {vn: new Vector(0, 0), vt: new Vector(0, 0)};
        this._collidable = true;
        this._multisample = 0;
        this.curr = new Vector(x, y);
        this.prev = new Vector(x, y);
        this._fixed = fixed;
        this._mass = mass;
        this._invMass = fixed ? 0 : 1 / mass;
        this._elasticity = elasticity;
        this._friction = friction;
    },
    get_mass: function() { return this._mass; },
    set_mass: function(m) {
        if (m <= 0) {
            throw new ArgumentError("mass may not be set <= 0");
        }
        this._mass = m;
        this._invMass = fixed ? 0 : 1 / this._mass;
    },
    get_invMass: function() { return this._invMass; },
    get_elasticity: function() { return this._elasticity; },
    set_elasticity: function(k) { this._elasticity = k; },
    get_multisample: function() { return this._multisample; },
    set_multisample: function(m) { this._multisample = m; },
    get_friction: function() { return this._friction; },
    set_friction: function(f) {
        if (f < 0 || f > 1)
            throw new ArgumentError("Legal friction must be >= 0 and <=1");
        this._friction = f;
    },
    get_fixed: function() { return this._fixed; },
    set_fixed: function(f) { this._fixed = f; },
    get_position: function() {
        return new Vector(this.curr.x, this.curr.y);
    },
    set_position: function(p) {
        this.curr.copy(p);
        this.prev.copy(p);
    },
    get_x: function() { return this.curr.x; },
    set_x: function(x) {
        this.curr.x = x;
        this.prev.x = x;
    },
    get_y: function() { return this.curr.y; },
    set_y: function(y) {
        this.curr.y = y;
        this.prev.y = y;
    },
    get_velocity: function() { return this.curr.minus(this.prev); },
    set_velocity: function(v) { this.prev = this.curr.minus(v); },
    get_collidable: function() { return this._collidable; },
    set_collidable: function(b) { this._collidable = b; },
    addForce: function(f) {
        this.forces.plusEquals(f.mult(this.get_invMass()));
    },
    addMasslessForce: function(f) {
        this.forces.plusEquals(f);
    },
    update: function(dt2, engine) {
        if (this.get_fixed()) {
            return;
        }
        this.addForce(engine.force);
        this.addMasslessForce(engine.gravity);
        var temp = new Vector(this.curr.x, this.curr.y);
        var nv = this.get_velocity().plus(this.forces.multEquals(dt2));
        this.curr.plusEquals(nv.multEquals(engine.damping));
        this.prev.copy(temp);
        this.forces.setTo(0, 0);
    },
    getComponents: function(collisionNormal) {
        var vel = this.get_velocity();
        var vdotn = collisionNormal.dot(vel);
        this.collision.vn = collisionNormal.mult(vdotn);
        this.collision.vt = vel.minus(this.collision.vn);
        return this.collision;
    },
    resolveCollision: function(mtd, vel, n, d, o, p) {
        this.curr.plusEquals(mtd);
        this.set_velocity(vel);
    },
    updatePosition: function() {},
    paint: function() {},
    toString: function() {
      return "Particle()";
    }
});
var Circle = Class.create(Particle, {
  initialize: function($super, x, y, radius, fixed, mass, elasticity, friction) {
    fixed = fixed || false;
    mass = mass || 1;
    elasticity = elasticity || 0.3;
    friction = friction || 0;
    $super(x, y, fixed, mass, elasticity, friction);
    this.radius = radius;
  },
  getProjection: function(axis) {
    var c = this.samp.dot(axis);
    this.interval.min = c - this.radius;
    this.interval.max = c + this.radius;
    return this.interval;
  },
  getIntervalX: function() {
    this.interval.min = this.curr.x - this.radius;
    this.interval.max = this.curr.x + this.radius;
    return this.interval;
  },
  getIntervalY: function() {
    this.interval.min = this.curr.y - this.radius;
    this.interval.max = this.curr.y + this.radius;
    return this.interval;
  },
  toString: function() {
    return "Circle("+this.x+", "+this.y+", "+this.radius+")";
  }
});
var Rect = Class.create(Particle, {
  initialize: function($super, x, y, width, height, rotation, fixed, mass, elasticity, friction) {
    rotation = rotation || 0;
    fixed = fixed || false;
    mass = mass || 1;
    elasticity = elasticity || 0.3;
    friction = friction || 0;
    $super(x, y, fixed, mass, elasticity, friction);
    this._axes = [new Vector(0, 0), new Vector(0, 0)];
    this._extents = [width / 2, height / 2];
    this.set_radian(rotation);
  },
  get_radian: function() { return this._radian; },
  set_radian: function(t) {
    this._radian = t;
    this.setAxes(t);
  },
  get_angle: function() {
    return this.get_radian() * (180 / Math.PI);
  },
  set_angle: function(a) {
    this.set_radian(a * (Math.PI / 180));
  },
  set_width: function(w) { this._extents[0] = w / 2; },
  get_width: function() { return this._extents[0] * 2; },
  set_height: function(h) { this._extents[1] = h / 2; },
  get_height: function() { return this._extents[1] * 2; },
  get_axes: function() { return this._axes; },
  get_extents: function() { return this._extents; },
  getProjection: function(axis) {
    var radius =
      this._extents[0] * Math.abs(axis.dot(this._axes[0]))+
      this._extents[1] * Math.abs(axis.dot(this._axes[1]));
    var c = this.samp.dot(axis);
    this.interval.min = c - radius;
    this.interval.max = c + radius;
    return this.interval;
  },
  setAxes: function(t) {
    var s = Math.sin(t);
    var c = Math.cos(t);
    this._axes[0].x = c;
    this._axes[0].y = s;
    this._axes[1].x = -s;
    this._axes[1].y = c;
  },
  toString: function() {
    return "Rect("+this.x+","+this.y+","+this.get_width()+","+this.get_height()+")";
  }
});
var Constraint = Class.create({
  initialize: function(stiffness) { this.stiffness = stiffness; },
  set_collidableParticle: function(p) {},
  get_collidableParticle: function() { return null; },
  isConnectedTo: function(p) { return false; },
  resolve: function() {},
  paint: function() {},
  toString: function() {
    return "Constraint("+stiffness+")";
  }
});
var Spring = Class.create(Constraint, {
  initialize: function($super, p1, p2, stiffness) {
    stiffness = stiffness || 0.5;
    $super(stiffness);
    this.p1 = p1;
    this.p2 = p2;
    this.checkParticlesLocation();
    this._restLength = this.get_currLength();
    this._scp = null;
  },
  get_radian: function() {
    var d = this.get_delta();
    return Math.atan2(d.y, d.x);
  },
  get_angle: function() {
    return this.get_radian() * (180 / Math.PI);
  },
  get_center: function() {
    return (this.p1.curr.plus(this.p2.curr)).divEquals(2);
  },
  set_rectScale: function(s) {
    if (this._scp != null) {
      this._scp.rectScale = s;
    }
  },
  get_rectScale: function() {
    if (this._scp != null) {
      return this._scp.rectScale;
    }
    return NaN;
  },
  get_currLength: function() {
    return this.p1.curr.distance(this.p2.curr);
  },
  get_rectHeight: function() {
    if (this._scp != null) {
      return this._scp.rectHeight;
    }
    return NaN;
  },
  set_rectHeight: function(h) {
    if (this._scp != null) {
      this._scp.rectHeight = h;
    }
  },
  get_restLength: function() { return this._restLength; },
  set_restLength: function(r) {
    if (r <= 0) {
      throw new ArgumentError("restLength must be greater than 0");
    }
    this._restLength = r;
  },
  get_fixedEndLimit: function() {
    if (this._scp != null) {
      return this._scp.fixedEndLimit;
    }
    return NaN;
  },
  set_fixedEndLimit: function(f) {
    if (this._scp != null) {
      this._scp.fixedEndLimit = f;
    }
  },
  set_collidableParticle: function(p) { this._scp = p; },
  get_collidableParticle: function() { return this._scp; },
  isConnectedTo: function(p) {
    return (p == this.p1 || p == this.p2);
  },
  get_fixed: function() {
    return (this.p1.get_fixed() && this.p2.get_fixed());
  },
  get_delta: function() {
    return this.p1.curr.minus(this.p2.curr);
  },
  resolve: function() {
    if (this.p1.get_fixed() && this.p2.get_fixed()) {
      return;
    }
    var deltaLength = this.get_currLength();
    var diff = (deltaLength - this.get_restLength()) / (deltaLength * (this.p1.get_invMass() + this.p2.get_invMass()));
    var dmds = this.get_delta().mult(diff * this.stiffness);
    this.p1.curr.minusEquals(dmds.mult(this.p1.get_invMass()));
    this.p2.curr.plusEquals (dmds.mult(this.p2.get_invMass()));
  },
  checkParticlesLocation: function() {
    if (this.p1.curr.x == this.p2.curr.x && this.p1.curr.y == this.p2.curr.y) {
      this.p2.curr.x += 0.0001;
    }
  },
  paint: function() {
    if (this._scp != null) {
      this._scp.paint();
    }
  },
  toString: function() {
    return "Spring()";
  }
});
var Collection = Class.create({
  initialize: function() {
    this.particles = [];
    this.constraints = [];
  },
  integrate: function(dt2, engine) {
    var a = this.particles;
    var l = a.length;
    for (var i = 0; i < l; ++i) {
      var p = a[i];
      p.update(dt2, engine);
    }
  },
  satisfyConstraints: function() {
    var a = this.constraints;
    var l = a.length;
    for (var i = 0; i < l; ++i) {
      var c = a[i];
      c.resolve();
    }
  },
  checkInternalCollisions: function() {
    var a1 = this.particles;
    var l1 = a1.length;
    var a2 = this.constraints;
    var l2 = a2.length;
    for (var j = 0; j < l1; j++) {
      var pj = a1[j];
      if (! pj.get_collidable()) {
        continue;
      }
      for (var i = j + 1; i < l1; i++) {
        var pi = a1[i];
        if (pi.get_collidable()) {
          collisionTest(pj, pi);
        }
      }
      for (var k = 0; k < l2; ++k) {
        var c = a2[k];
        var pk = c.get_collidableParticle();
        if (pk != null && ! c.isConnectedTo(pj)) {
          pk.updatePosition();
          collisionTest(pj, pk);
        }
      }
    }
  },
  checkCollisionsVsCollection: function(that) {
    var a1 = this.particles;
    var l1 = a1.length;
    var a2 = that.particles;
    var l2 = a2.length;
    var a3 = this.constraints;
    var l3 = a3.length;
    var a4 = that.constraints;
    var l4 = a4.length;
    for (var i1 = 0; i1 < l1; ++i1) {
      var pga = a1[i1];
      if (! pga.get_collidable()) {
        continue;
      }
      for (var i2 = 0; i2 < l2; ++i2) {
        var pgb = a2[i2];
        if (pgb.get_collidable()) {
          collisionTest(pga, pgb);
        }
      }
      for (var i4 = 0; i4 < l4; ++i4) {
        var cgb = a4[i4];
        var pk = cgb.get_collidableParticle();
        if (pk != null && ! cgb.isConnectedTo(pga)) {
          pk.updatePosition();
          collisionTest(pga, pk);
        }
      }
    }
    for (var i3 = 0; i3 < l3; ++i3) {
      var cga = a3[i3];
      var pl = cga.get_collidableParticle();
      if (pl == null) {
        continue;
      }
      for (i2 = 0; i2 < l2; ++i2) {
        pgb = a2[i2];
        if (pgb.get_collidable() && !cga.isConnectedTo(pgb)) {
          pl.updatePosition();
          collisionTest(pgb, pl);
        }
      }
    }
  },
  paint: function() {
    var a1 = this.particles;
    var l1 = a1.length;
    for (var i1 = 0; i1 < l1; ++i1) {
      var p = a1[i1];
      p.paint();
    }
    var a2 = this.constraints;
    var l2 = a2.length;
    for (var i2 = 0; i2 < l2; ++i2) {
      var c = a2[i2];
      c.paint();
    }
  },
  toString: function() {
    return "Collection()";
  }
});
var Composite = Class.create(Collection, {
  initialize: function($super) {
    $super();
  },
  rotate: function(angleRadians, center) {
    var a = this.particles;
    var l = a.length;
    for (var i = 0; i < l; ++i) {
      var p = a[i];
      var other = p.get_position();
      var radius = other.distance(center);
      var angle = Math.atan2(other.y - center.y, other.x - center.x) + angleRadians;
      p.set_x((Math.cos(angle) * radius) + center.x);
      p.set_y((Math.sin(angle) * radius) + center.y);
    }
  },
  toString: function() {
    return "Composite()";
  }
});
var Group = Class.create(Collection, {
  initialize: function($super, collideInternal) {
    $super();
    this.collideInternal = collideInternal || false;
    this.composites = [];
    this.collisionList = [];
  },
  integrate: function($super, dt2, engine) {
    $super(dt2, engine);
    var a = this.composites;
    var l = a.length;
    for (var i = 0; i < l; ++i) {
      var cmp = a[i];
      cmp.integrate(dt2, engine);
    }
  },
  satisfyConstraints: function($super) {
    $super();
    var a = this.composites;
    var l = a.length;
    for (var i = 0; i < l; ++i) {
      var cmp = a[i];
      cmp.satisfyConstraints();
    }
  },
  checkCollisions: function() {
    if (this.collideInternal) {
      this.checkCollisionGroupInternal();
    }
    var a = this.collisionList;
    var l = a.length;
    for (var i = 0; i < l; ++i) {
      var g = a[i];
      this.checkCollisionVsGroup(g);
    }
  },
  checkCollisionGroupInternal: function() {
    this.checkInternalCollisions();
    var clen = this.composites.length;
    for (var j = 0; j < clen; j++) {
      var cj = this.composites[j];
      cj.checkCollisionsVsCollection(this);
      for (var i = j + 1; i < clen; i++) {
        var ci = this.composites[i];
        cj.checkCollisionsVsCollection(ci);
      }
    }
  },
  checkCollisionVsGroup: function(that) {
    this.checkCollisionsVsCollection(that);
    var a1 = this.composites;
    var l1 = a1.length;
    var a2 = that.composites;
    var l2 = a2.length;
    for (var i = 0; i < l1; ++i) {
      var c = a1[i];
      c.checkCollisionsVsCollection(that);
      for (var j = 0; j < l2; ++j) {
        var gc = a2[j];
        c.checkCollisionsVsCollection(gc);
      }
    }
    for (j = 0; j < l2; ++j) {
      gc = a2[j];
      this.checkCollisionsVsCollection(gc);
    }
  },
  paint: function($super) {
    $super();
    var a = this.composites;
    var l = a.length;
    for (var i = 0; i < l; ++i) {
      var c = a[i];
      c.paint();
    }
  },
  toString: function() {
    return "Group()";
  }
});
/**
 * Tests the collision between two objects. If there is a collision it is passed off
 * to the CollisionResolver class.
 */
function collisionTest(objA, objB) {
    if (objA.get_fixed() && objB.get_fixed()) {
        return;
    }
    if (objA.get_multisample() == 0 && objB.get_multisample() == 0) {
        normVsNorm(objA, objB);
    } else if (objA.get_multisample() > 0 && objB.get_multisample() == 0) {
        sampVsNorm(objA, objB);
    } else if (objB.get_multisample() > 0 && objA.get_multisample() == 0) {
        sampVsNorm(objB, objA);
    } else if (objA.get_multisample() == objB.get_multisample()) {
        sampVsSamp(objA, objB);
    } else {
        normVsNorm(objA, objB);
    }
}

/**
 * default test for two non-multisampled particles
 */
function normVsNorm(objA, objB) {
    objA.samp.copy(objA.curr);
    objB.samp.copy(objB.curr);
    testTypes(objA, objB);
}

/**
 * Tests two particles where one is multisampled and the other is not. Let objectA
 * be the multisampled particle.
 */
function sampVsNorm(objA, objB) {
    var s = 1 / (objA.get_multisample() + 1);
    var t = s;
    objB.samp.copy(objB.curr);
    for (var i = 0; i <= objA.get_multisample(); i++) {
        objA.samp.setTo(objA.prev.x + t * (objA.curr.x - objA.prev.x),
                        objA.prev.y + t * (objA.curr.y - objA.prev.y));
        if (testTypes(objA, objB)) {
            return;
        }
        t += s;
    }
}

/**
 * Tests two particles where both are of equal multisample rate
 */
function sampVsSamp(objA, objB) {
    var s = 1 / (objA.get_multisample() + 1);
    var t = s;
    for (var i = 0; i <= objA.get_multisample(); i++) {
        objA.samp.setTo(objA.prev.x + t * (objA.curr.x - objA.prev.x),
                        objA.prev.y + t * (objA.curr.y - objA.prev.y));
        objB.samp.setTo(objB.prev.x + t * (objB.curr.x - objB.prev.x),
                        objB.prev.y + t * (objB.curr.y - objB.prev.y));
        if (testTypes(objA, objB)) {
            return;
        }
        t += s;
    }
}

/**
 */
function testTypes(objA, objB) {
  if ((objA instanceof Circle) && (objB instanceof Circle)) {
    testCirclevsCircle(objA, objB);
  } else if ((objA instanceof Circle) && (objB instanceof Rect)) {
    testCirclevsOBB(objA, objB);
  } else if ((objA instanceof Rect) && (objB instanceof Circle)) {
    testOBBvsCircle(objA, objB);
  } else {
    testOBBvsOBB(objA, objB);
  }
}

/**
 * Tests the collision between two Rect (aka OBBs). If there is a collision it
 * determines its axis and depth, and then passes it off to the CollisionResolver for handling.
 */
function testOBBvsOBB(ra, rb) {
    var collisionNormal;
    var collisionDepth = Number.POSITIVE_INFINITY;
    for (var i = 0; i < 2; i++) {
        var axisA = ra.get_axes()[i];
        var depthA = testIntervals(ra.getProjection(axisA), rb.getProjection(axisA));
        if (depthA == 0) {
            return false;
        }
        var axisB = rb.get_axes()[i];
        var depthB = testIntervals(ra.getProjection(axisB), rb.getProjection(axisB));
        if (depthB == 0) {
            return false;
        }
        var absA = Math.abs(depthA);
        var absB = Math.abs(depthB);
        if (absA < Math.abs(collisionDepth) || absB < Math.abs(collisionDepth)) {
            var altb = absA < absB;
            collisionNormal = altb ? axisA : axisB;
            collisionDepth = altb ? depthA : depthB;
        }
    }
    resolveParticleParticle(ra, rb, collisionNormal, collisionDepth);
    return true;
}

function testCirclevsOBB(ca, ra) {
    return testOBBvsCircle(ra, ca);
}

/**
 * Tests the collision between a Rect (aka an OBB) and a Circle.
 * If there is a collision it determines its axis and depth, and then passes it off
 * to the CollisionResolver.
 */
function testOBBvsCircle(ra, ca) {
    var collisionNormal;
    var collisionDepth = Number.POSITIVE_INFINITY;
    var depths = new Array(2);
    // first go through the axes of the rectangle
    for (var i = 0; i < 2; i++) {
        var boxAxis = ra.get_axes()[i];
        var depth = testIntervals(ra.getProjection(boxAxis), ca.getProjection(boxAxis));
        if (depth == 0) {
            return false;
        }
        if (Math.abs(depth) < Math.abs(collisionDepth)) {
            collisionNormal = boxAxis;
            collisionDepth = depth;
        }
        depths[i] = depth;
    }
    // determine if the circle's center is in a vertex region
    var r = ca.radius;
    if (Math.abs(depths[0]) < r && Math.abs(depths[1]) < r) {
        var vertex = closestVertexOnOBB(ca.samp, ra);
        // get the distance from the closest vertex on rect to circle center
        collisionNormal = vertex.minus(ca.samp);
        var mag = collisionNormal.magnitude();
        collisionDepth = r - mag;
        if (collisionDepth > 0) {
            // there is a collision in one of the vertex regions
            collisionNormal.divEquals(mag);
        } else {
            // ra is in vertex region, but is not colliding
            return false;
        }
    }
    resolveParticleParticle(ra, ca, collisionNormal, collisionDepth);
    return true;
}

/**
 * Tests the collision between two Circles. If there is a collision it
 * determines its axis and depth, and then passes it off to the CollisionResolver
 * for handling.
 */
function testCirclevsCircle(ca, cb) {
    var depthX = testIntervals(ca.getIntervalX(), cb.getIntervalX());
    if (depthX == 0) {
        return false;
    }
    var depthY = testIntervals(ca.getIntervalY(), cb.getIntervalY());
    if (depthY == 0) {
        return false;
    }
    var collisionNormal = ca.samp.minus(cb.samp);
    var mag = collisionNormal.magnitude();
    var collisionDepth = (ca.radius + cb.radius) - mag;
    if (collisionDepth > 0) {
        collisionNormal.divEquals(mag);
        resolveParticleParticle(ca, cb, collisionNormal, collisionDepth);
        return true;
    }
    return false;
}

/**
 * Returns 0 if intervals do not overlap. Returns smallest depth if they do.
 */
function testIntervals(intervalA, intervalB) {
    if (intervalA.max < intervalB.min) {
        return 0;
    }
    if (intervalB.max < intervalA.min) {
        return 0;
    }
    var lenA = intervalB.max - intervalA.min;
    var lenB = intervalB.min - intervalA.max;
    return (Math.abs(lenA) < Math.abs(lenB)) ? lenA : lenB;
}

/**
 * Returns the location of the closest vertex on r to point p
 */
function closestVertexOnOBB(p, r) {
    var d = p.minus(r.samp);
    var q = new Vector(r.samp.x, r.samp.y);
    for (var i = 0; i < 2; i++) {
        var dist = d.dot(r.get_axes()[i]);
        if (dist >= 0) {
            dist = r.get_extents()[i];
        } else if (dist < 0) {
            dist = -r.get_extents()[i];
        }
        q.plusEquals(r.get_axes()[i].mult(dist));
    }
    return q;
}
/**
 * @return n clamped between 0 and 1
 */
function clamp01(n) {
  if (n < 0)
    return 0;
  if (n > 1)
    return 1;
  return n;
}

// thanks to Jim Bonacci for changes using the inverse mass instead of mass
function resolveParticleParticle(pa, pb, normal, depth) {
    // a collision has occured. set the current positions to sample locations
    pa.curr.copy(pa.samp);
    pb.curr.copy(pb.samp);
    var mtd = normal.mult(depth);
    var te = pa.get_elasticity() + pb.get_elasticity();
    var sumInvMass = pa.get_invMass() + pb.get_invMass();
    // the total friction in a collision is combined but clamped to [0,1]
    var tf = clamp01(1 - (pa.get_friction() + pb.get_friction()));
    // get the collision, vn and vt
    var ca = pa.getComponents(normal);
    var cb = pb.getComponents(normal);
    // calculate the coefficient of restitution based on the mass, as the normal component
    var vnA = (cb.vn.mult((te + 1) * pa.get_invMass()).plus(ca.vn.mult(pb.get_invMass() - te * pa.get_invMass()))).divEquals(sumInvMass);
    var vnB = (ca.vn.mult((te + 1) * pb.get_invMass()).plus(cb.vn.mult(pa.get_invMass() - te * pb.get_invMass()))).divEquals(sumInvMass);
    // apply friction to the tangental component
    ca.vt.multEquals(tf);
    cb.vt.multEquals(tf);
    // scale the mtd by the ratio of the masses. heavier particles move less
    var mtdA = mtd.mult( pa.get_invMass() / sumInvMass);
    var mtdB = mtd.mult(-pb.get_invMass() / sumInvMass);
    // add the tangental component to the normal component for the new velocity
    vnA.plusEquals(ca.vt);
    vnB.plusEquals(cb.vt);
    if (! pa.get_fixed()) {
        pa.resolveCollision(mtdA, vnA, normal, depth, -1, pb);
    }
    if (! pb.get_fixed()) {
        pb.resolveCollision(mtdB, vnB, normal, depth,  1, pa);
    }
}
/**
 * The Rim is really just a second component of the wheel model.
 * The rim particle is simulated in a coordsystem relative to the wheel's
 * center, not in worldspace.
 *
 * Origins of this code are from Raigan Burns, Metanet Software
 */
var Rim = Class.create({
  initialize: function(r, mt, parent) {
    this.curr = new Vector(r, 0);
    this.prev = new Vector(0, 0);
    this.sp = 0;
    this.av = 0;
    this.maxTorque = mt;
    this.wr = r;
    this.parent = parent;
  },
  get_speed: function() { return this.sp; },
  set_speed: function(s) { this.sp = s; },
  get_angularVelocity: function() { return this.av; },
  set_angularVelocity: function(s) { this.av = s; },
  /**
   * Origins of this code are from Raigan Burns, Metanet Software
   */
  update: function(dt, engine) {
    //clamp torques to valid range
    this.sp = Math.max(-this.maxTorque, Math.min(this.maxTorque, this.sp + this.av));
    //apply torque
    //this is the tangent vector at the rim particle
    var dx = -this.curr.y;
    var dy =  this.curr.x;
    //normalize so we can scale by the rotational speed
    var len = Math.sqrt(dx * dx + dy * dy);
    dx /= len;
    dy /= len;
    this.curr.x += this.sp * dx;
    this.curr.y += this.sp * dy;
    var ox = this.prev.x;
    var oy = this.prev.y;
    var px = this.prev.x = this.curr.x;
    var py = this.prev.y = this.curr.y;
    this.curr.x += engine.damping * (px - ox);
    this.curr.y += engine.damping * (py - oy);
    // hold the rim particle in place
    var clen = Math.sqrt(this.curr.x * this.curr.x + this.curr.y * this.curr.y);
    var diff = (clen - this.wr) / clen;
    this.curr.x -= this.curr.x * diff;
    this.curr.y -= this.curr.y * diff;
  },
  toString: function() {
    return "Rim()";
  }
});
/**
 * Returns 1 if the value is >= 0. Returns -1 if the value is < 0.
 */
function sign(val) {
  return (val < 0) ? -1 : 1;
}

var Wheel = Class.create(Circle, {
  initialize: function($super, x, y, radius, fixed, mass, elasticity, friction, traction) {
    fixed = fixed || false;
    mass = mass || 1;
    elasticity = elasticity || 0.3;
    friction = friction || 0;
    traction = traction || 1;
    $super(x, y, radius, fixed, mass, elasticity, friction);
    this.tan = new Vector();
    this.normSlip = new Vector();
    this.orientation = new Vector();
    this.rim = new Rim(radius, 2, this);
    this.set_traction(traction);
  },
  get_speed: function() { return this.rim.get_speed(); },
  set_speed: function(s) { this.rim.set_speed(s); },
  get_angularVelocity: function() { return this.rim.get_angularVelocity(); },
  set_angularVelocity: function(a) { this.rim.set_angularVelocity(a); },
  get_traction: function() { return 1 - this._traction; },
  set_traction: function(t) { this._traction = 1 - t; },
  get_radian: function() {
    this.orientation.setTo(this.rim.curr.x, this.rim.curr.y);
    return Math.atan2(this.orientation.y, this.orientation.x) + Math.PI;
  },
  get_angle: function() {
    return this.get_radian() * (180 / Math.PI);
  },
  update: function($super, dt, engine) {
    $super(dt, engine);
    this.rim.update(dt, engine);
  },
  resolveCollision: function($super, mtd, vel, n, d, o, p) {
    // review the o (order) need here - its a hack fix
    $super(mtd, vel, n, d, o, p);
    this.resolve(n.mult(sign(d * o)));
  },
  resolve: function(n) {
    // this is the tangent vector at the this.rim particle
    this.tan.setTo(-this.rim.curr.y, this.rim.curr.x);
    // normalize so we can scale by the rotational speed
    this.tan = this.tan.normalize();
    // velocity of the wheel's surface
    var wheelSurfaceVelocity = this.tan.mult(this.rim.get_speed());
    // the velocity of the wheel's surface relative to the ground
    var combinedVelocity = this.get_velocity().plusEquals(wheelSurfaceVelocity);
    // the wheel's comb velocity projected onto the contact normal
    var cp = combinedVelocity.cross(n);
    // set the wheel's spinspeed to track the ground
    this.tan.multEquals(cp);
    this.rim.prev.copy(this.rim.curr.minus(this.tan));
    // some of the wheel's torque is removed and converted into linear displacement
    var slipSpeed = (1 - this._traction) * this.rim.get_speed();
    this.normSlip.setTo(slipSpeed * n.y, slipSpeed * n.x);
    this.curr.plusEquals(this.normSlip);
    this.rim.set_speed(this.rim.get_speed() * this._traction);
  },
  toString: function() {
    return "Wheel()";
  }
});
var SpringRect = Class.create(Rect, {
  initialize: function($super, spring, rectHeight, rectScale, scaleToLength) {
    $super(0, 0, 0, 0, 0, false);
    this._spring = spring;
    this.p1 = spring.p1;
    this.p2 = spring.p2;
    this.rectScale = rectScale || 1;
    this.rectHeight = rectHeight || 1;
    this.scaleToLength = scaleToLength || false;
    this.avgVelocity = new Vector();
    this.lambda = new Vector();
    this.rca = new Vector();
    this.rcb = new Vector();
    this.fixedEndLimit = 0;
  },
  get_mass: function() { return (this.p1.get_mass() + this.p2.get_mass()) / 2; },
  get_elasticity: function() { return (this.p1.get_elasticity() + this.p2.get_elasticity()) / 2; },
  get_friction: function() { return (this.p1.get_friction() + this.p2.get_friction()) / 2; },
  get_velocity: function() {
    var p1v =  this.p1.get_velocity();
    var p2v =  this.p2.get_velocity();
    this.avgVelocity.setTo(((p1v.x + p2v.x) / 2), ((p1v.y + p2v.y) / 2));
    return this.avgVelocity;
  },
  get_invMass: function() {
    if (this.p1.get_fixed() && this.p2.get_fixed()) {
      return 0;
    }
    return 1 / ((this.p1.get_mass() + this.p2.get_mass()) / 2);
  },
  updatePosition: function() {
    var c = this._spring.get_center();
    this.curr.setTo(c.x, c.y);
    this.set_width(this.scaleToLength ? this._spring.get_currLength() * this.rectScale : this._spring.get_restLength() * this.rectScale);
    this.set_height(this.rectHeight);
    this.set_radian(this._spring.get_radian());
  },
  resolveCollision: function(mtd, vel, n, d, o, p) {
    var t = this.getContactPointParam(p);
    var c1 = (1 - t);
    var c2 = t;
    // if one is fixed then move the other particle the entire way out of collision.
    // also, dispose of collisions at the sides of the scp. The higher the this.fixedEndLimit
    // value, the more of the scp not be effected by collision.
    if (this.p1.get_fixed()) {
      if (c2 <= this.fixedEndLimit) {
        return;
      }
      this.lambda.setTo(mtd.x / c2, mtd.y / c2);
      this.p2.curr.plusEquals(this.lambda);
      this.p2.set_velocity(vel);
    } else if (this.p2.get_fixed()) {
      if (c1 <= this.fixedEndLimit) {
        return;
      }
      this.lambda.setTo(mtd.x / c1, mtd.y / c1);
      this.p1.curr.plusEquals(this.lambda);
      this.p1.set_velocity(vel);
      // else both non fixed - move proportionally out of collision
    } else {
      var denom = (c1 * c1 + c2 * c2);
      if (denom == 0) {
        return;
      }
      this.lambda.setTo(mtd.x / denom, mtd.y / denom);
      this.p1.curr.plusEquals(this.lambda.mult(c1));
      this.p2.curr.plusEquals(this.lambda.mult(c2));
      // if collision is in the middle of SCP set the velocity of both end particles
      if (t == 0.5) {
        this.p1.set_velocity(vel);
        this.p2.set_velocity(vel);
        // otherwise change the velocity of the particle closest to contact
      } else {
        var corrParticle = (t < 0.5) ? this.p1 : this.p2;
        corrParticle.set_velocity(vel);
      }
    }
  },
  closestParamPoint: function(c) {
    var ab = this.p2.curr.minus(this.p1.curr);
    var t = (ab.dot(c.minus(this.p1.curr))) / (ab.dot(ab));
    return clamp01(t);
  },
  getContactPointParam: function(p) {
    var t;
    if (p instanceof Circle)  {
      t = this.closestParamPoint(p.curr);
    } else if (p instanceof Rect) {
      // go through the sides of the colliding rectangle as line segments
      var shortestIndex;
      var paramList = new Array(4);
      var shortestDistance = Number.POSITIVE_INFINITY;
      for (var i = 0; i < 4; i++) {
        this.setCorners(p, i);
        // check for closest points on SCP to side of rectangle
        var d = this.closestPtSegmentSegment();
        if (d < shortestDistance) {
          shortestDistance = d;
          shortestIndex = i;
          paramList[i] = this.s;
        }
      }
      t = paramList[shortestIndex];
    }
    return t;
  },
  setCorners: function(r, i) {
    var rx = r.curr.x;
    var ry = r.curr.y;
    var axes = r.get_axes();
    var extents = r.get_extents();
    var ae0_x = axes[0].x * extents[0];
    var ae0_y = axes[0].y * extents[0];
    var ae1_x = axes[1].x * extents[1];
    var ae1_y = axes[1].y * extents[1];
    var emx = ae0_x - ae1_x;
    var emy = ae0_y - ae1_y;
    var epx = ae0_x + ae1_x;
    var epy = ae0_y + ae1_y;
    if (i == 0) {
      // 0 and 1
      this.rca.x = rx - epx;
      this.rca.y = ry - epy;
      this.rcb.x = rx + emx;
      this.rcb.y = ry + emy;
    } else if (i == 1) {
      // 1 and 2
      this.rca.x = rx + emx;
      this.rca.y = ry + emy;
      this.rcb.x = rx + epx;
      this.rcb.y = ry + epy;
    } else if (i == 2) {
      // 2 and 3
      this.rca.x = rx + epx;
      this.rca.y = ry + epy;
      this.rcb.x = rx - emx;
      this.rcb.y = ry - emy;
    } else if (i == 3) {
      // 3 and 0
      this.rca.x = rx - emx;
      this.rca.y = ry - emy;
      this.rcb.x = rx - epx;
      this.rcb.y = ry - epy;
    }
  },
  closestPtSegmentSegment: function() {
    var pp1 = this.p1.curr;
    var pq1 = this.p2.curr;
    var pp2 = this.rca;
    var pq2 = this.rcb;
    var d1 = pq1.minus(pp1);
    var d2 = pq2.minus(pp2);
    var r = pp1.minus(pp2);
    var t;
    var a = d1.dot(d1);
    var e = d2.dot(d2);
    var f = d2.dot(r);
    var c = d1.dot(r);
    var b = d1.dot(d2);
    var denom = a * e - b * b;
    if (denom != 0.0) {
      this.s = clamp01((b * f - c * e) / denom);
    } else {
      this.s = 0.5; // give the midpoint for parallel lines
    }
    t = (b * this.s + f) / e;
    if (t < 0) {
      t = 0;
      this.s = clamp01(-c / a);
    } else if (t > 0) {
      t = 1;
      this.s = clamp01((b - c) / a);
    }
    var c1 = pp1.plus(d1.mult(this.s));
    var c2 = pp2.plus(d2.mult(t));
    var c1mc2 = c1.minus(c2);
    return c1mc2.dot(c1mc2);
  },
  toString: function() {
    return "SpringRect()";
  }
});
/** Dynamics Engine
 */
var Engine = Class.create({
/**
 * Initializes the engine. You must call this method prior to adding
 * any particles or constraints.
 *
 * @param dt The delta time value for the engine. This
 * parameter can be used -- in conjunction with speed at which
 * <code>Engine.step()</code> is called -- to change the speed
 * of the simulation. Typical values are 1/3 or 1/4. Lower
 * values result in slower, but more accurate simulations, and
 * higher ones result in faster, less accurate ones. Note
 * that this only applies to the forces added to particles. If
 * you do not add any forces, the <code>dt</code> value won't
 * matter.
 */
  initialize: function(dt) {
    dt = dt || 0.25;
    /** 重いものほど動かない力 例:風が吹く */
    this.force = new Vector(0, 0);
    /** 質量に関係なく働く力 例:重力 */
    this.gravity = new Vector(0, 0);
    /** グループ Group の配列 */
    this.groups = [];
    /** delta time の二乗 */
    this._timeStep = dt * dt;
    /** 制動値 [0, 1] 1は制動しない。0は全粒子が動かない */
    this.damping = 1;
    /** 制約解消する回数 増やすと硬くなる */
    this.constraintCycles = 0;
    /** 衝突解消する回数 増やすと安定する */
    this.constraintCollisionCycles = 1;
  },
/**
 * The main step of: function the engine. This method should be
 * called continously to advance the simulation. The faster
 * this method is called, the faster the simulation will
 * run. Usually you would call this in your main program loop.
 */
  step: function() {
    var a = this.groups;
    var l = a.length;
    for (var i = 0; i < l; ++i) {
      var g = a[i];
      g.integrate(this._timeStep, this);
    }
    for (var j = 0; j < this.constraintCycles; j++) {
      for (i = 0; i < l; ++i) {
        g = a[i];
        g.satisfyConstraints();
      }
    }
    for (j = 0; j < this.constraintCollisionCycles; j++) {
      for (i = 0; i < l; ++i) {
        g = a[i];
        g.satisfyConstraints();
      }
      for (i = 0; i < l; ++i) {
        g = a[i];
        g.checkCollisions();
      }
    }
  },
/**
 * Calling this method will in turn call each particle and
 * constraint's paint method.  Generally you would call this
 * method after stepping the engine in the main program cycle.
 */
  paint: function() {
    var a = this.groups;
    var l = a.length;
    for (var i = 0; i < l; ++i) {
      var g = a[i];
      g.paint();
    }
  }
});
