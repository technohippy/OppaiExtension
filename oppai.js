var Settings = {
  center: new Vector(0, 200),
  radius: 180,
};

var CanvasCircle = Class.create(Circle, {
  initialize: function($super, x, y, radius, fixed, mass, elasticity, friction, gc) {
    $super(x, y, radius, fixed, mass, elasticity, friction);
    this.gc = gc;
  },
 
  paint: function() {
    var c = {x:this.get_x(), y:this.get_y()};
    this.gc.moveTo(c.x, c.y);
    this.gc.arc(c.x, c.y, this.radius, 0, 2 * Math.PI, true);
  },
});

var CanvasLineSpring = Class.create(Spring, {
  initialize: function($super, p1, p2, stiffness, gc) {
    $super(p1, p2, stiffness);
    this.gc = gc;
  },
 
  paint: function() {
    this.gc.moveTo(this.p1.curr.x, this.p1.curr.y);
    this.gc.lineTo(this.p2.curr.x, this.p2.curr.y);
  },
});

var Oppai = Class.create(Group, {
  initialize: function($super, gc, pressure, maxDeg) {
    $super();
 
    this.gc = gc;
    this.pressure = pressure || 10;
    this.maxDeg = maxDeg || 90;
    this.initializePartices();
  },

  initializePartices: function() {
    var maxRad = this.maxDeg / 180.0 * Math.PI;
    var step = this.maxDeg * 2.0 / 10;
    var baseLength = 180;
    var centerY = 200;
    var radius = baseLength / Math.sin(maxRad);
    var centerX = -baseLength / Math.tan(maxRad);

    this.particles = [];
    this.constraints = [];
    var prevBody = null;
    for (var deg = -this.maxDeg; deg <= this.maxDeg; deg += step) {
      var rad = (deg / 180.0) * Math.PI;
      var x = radius * Math.cos(rad) + centerX;
      var y = radius * Math.sin(rad) + centerY;
      var fixed = Math.abs(deg) == Math.abs(this.maxDeg);
      var body = new CanvasCircle(x, y, 10, fixed, 1, 0.3, 0, this.gc);
      this.particles.push(body);
      if (prevBody) {
        var spring = new CanvasLineSpring(body, prevBody, 1, this.gc);
        this.constraints.push(spring);
      }
      prevBody = body;
    }
  },

  applyPressure: function() {
    for (var i = 1; i < this.particles.length - 1; i++) {
      var prev = this.particles[i-1];
      var curr = this.particles[i];
      var next = this.particles[i+1];
      var p2c = curr.get_position().minus(prev.get_position()).normalize();
      var p2n = next.get_position().minus(prev.get_position()).normalize();
      var f;
      if (0 < p2c.cross(p2n)) {
        var n2c = curr.get_position().minus(next.get_position()).normalize();
        f = p2c.add(n2c).normalize().mult(this.pressure);
      } else {
        var c2p = prev.get_position().minus(curr.get_position()).normalize();
        var c2n = next.get_position().minus(curr.get_position()).normalize();
        f = c2p.add(c2n).normalize().mult(this.pressure);
      }
      curr.addForce(f);
    }
  },

  wobble: function() {
    for (var i = 1; i < this.particles.length - 1; i++) {
      this.particles[i].addForce(new Vector(0, -100));
    }
  },

  paint: function() {
    var scale = 1.0 / 20; // 400 -> 20
    var first = this.particles[0].get_position();
    this.gc.moveTo(first.x * scale, first.y * scale);
    for (var i = 1; i < this.particles.length - 1; i++) {
      var prev = this.particles[i-1].get_position();
      var curr = this.particles[i].get_position();
      var next = this.particles[i+1].get_position();
      var pc = prev.add(curr).mult(0.5);
      var cn = curr.add(next).mult(0.5);
      if (i == 1) {
        this.gc.lineTo(pc.x * scale, pc.y * scale);
      }
      this.gc.quadraticCurveTo(curr.x * scale, curr.y * scale, cn.x * scale, cn.y * scale);
    }
    var last = this.particles[this.particles.length - 1].get_position();
    this.gc.lineTo(last.x * scale, last.y * scale);
  },

  setPressure: function(pressure) {
    this.pressure = pressure;
  },

  setMaxDeg: function(maxDeg) {
    this.maxDeg = maxDeg;
    this.initializePartices();
  },

  setSize: function(size) {
    var nextDeg = Math.floor(6 + 6 * size) * 10;
    if (this.maxDeg == nextDeg) return;
    this.setMaxDeg(nextDeg);
  },
});

var Wall = Class.create(Group, {
  initialize: function($super, gc) {
    $super();
    this.gc = gc;
    var radius = 90 * 1.41421356;
    var center = new Vector(-95, Settings.center.y);
    var body = new CanvasCircle(center.x, center.y, radius, true, 1, 0.3, 0, gc);
    this.particles.push(body);
  },
});

var Hand = Class.create(Group, {
  initialize: function($super, gc) {
    $super();
    for (var i = 0; i < 5; i++) {
      this.particles.push(new CanvasCircle(-100, -100 * i, 40, true, 1, 0.3, 0, gc));
    }
  },
 
  moveFingersTo: function(points) {
    for (var i = 0; i < this.particles.length; i++) {
      if (points[i]) {
        if (points[i].pageX) {
          var point = {x:points[i].pageX, y:points[i].pageY}; 
          this.particles[i].set_x(point.x);
          this.particles[i].set_y(point.y);
        } else {
          this.particles[i].set_x(points[i].x);
          this.particles[i].set_y(points[i].y);
        }
      } else {
        this.particles[i].set_x(-100);
        this.particles[i].set_y(-100 * i);
      }
    }
  },
});

var World = Class.create({
  initialize: function(canvasId) {
    this.gc = $(canvasId).getContext('2d');
    this.gc.strokeStyle = '#ffffff';
    this.gc.fillStyle = '#ffffff';
    this.oppai = new Oppai(this.gc);
    this.wall = new Wall(this.gc);
    this.wall.collisionList = [this.oppai];
    this.hand = new Hand(this.gc);
    this.hand.collisionList = [this.oppai];
    this.engine = new Engine(1/4);
    this.engine.gravity = new Vector(0, 3);
    this.engine.groups = [this.oppai, this.wall, this.hand];
  },

  moveFingersTo: function(points) {
    this.hand.moveFingersTo(points);
  },
 
  run: function() {
    //this.gc.clearRect(0, 0, 20, 20);

    this.engine.step();
    this.oppai.applyPressure();
 
    this.gc.beginPath();

    this.gc.fillStyle = 'rgb(0, 0, 0)';
    this.gc.fillRect(0, 0, 20, 20);
    this.gc.fillStyle = 'rgb(255, 255, 255)';
 
    this.oppai.paint();
    this.gc.closePath();
    this.gc.fill();
  },

  wobble: function() {
    this.oppai.wobble();
  }
});

var canvasId = 'canvas';
var world = null;
Event.observe(window, 'load', function(e) {
  world = new World(canvasId);
  world.run();
  setInterval(function() {
    world.run();
    var canvas = $('canvas');
    var context = canvas.getContext('2d');
    if (context ) {
      var imageData = context.getImageData(0, 0, 19, 19); 
      chrome.browserAction.setIcon({imageData: imageData});
    }
  }, 20);
}, true);

