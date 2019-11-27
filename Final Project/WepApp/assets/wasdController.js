//w a s d Controls

var up = false,
    right = false,
    down = false,
    left = false,
    x = window.innerWidth/2-130/2,
    y = window.innerHeight/2-130/2

document.addEventListener('keydown',press)
function press(e){
  if (e.keyCode === 38 /* up */ || e.keyCode === 87 /* w */){
    up = true
  }
  if (e.keyCode === 39 /* right */ || e.keyCode === 68 /* d */){
    right = true
  }
  if (e.keyCode === 40 /* down */ || e.keyCode === 83 /* s */){
    down = true
  }
  if (e.keyCode === 37 /* left */ || e.keyCode === 65 /* a */){
    left = true
  }
}
document.addEventListener('keyup',release)
function release(e){
  if (e.keyCode === 38 /* up */ || e.keyCode === 87 /* w */){
    up = false
  }
  if (e.keyCode === 39 /* right */ || e.keyCode === 68 /* d */){
    right = false
  }
  if (e.keyCode === 40 /* down */ || e.keyCode === 83 /* s */){
    down = false
  }
  if (e.keyCode === 37 /* left */ || e.keyCode === 65 /* a */){
    left = false
  }
}

function gameLoop(){
  var div = document.querySelector('div')
  if (up){
    y = y - 10
  }
  if (right){
    x = x + 10
  }
  if (down){
    y = y + 10
  }
  if (left){
    x = x - 10
  }
  div.style.left = x+'px'
  div.style.top = y+'px'
  window.requestAnimationFrame(gameLoop)
}
window.requestAnimationFrame(gameLoop)