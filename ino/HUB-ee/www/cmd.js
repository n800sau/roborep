function send_command(command) {
  var request = new XMLHttpRequest();
  request.onreadystatechange = function() {
    if(request.readyState == 4)
    {
      if(request.status == 200) {
        var jsonObj = JSON.parse(request.responseText);
        document.getElementById("reply").innerHTML = command + ':' + jsonObj.result;
      } else {
        document.getElementById("reply").innerHTML = command + ':' + request.responseText;
      }
    }
    setTimeout(reset_buttons, 1000);
  }
  request.open("GET", '/rgbframe/command.php?cmd=' + command, true);
  request.send();
}

function command(elm) {
  send_command(elm.id);
}

function reset_buttons() {
  document.getElementById("stop").checked = 'checked';
  document.getElementById("sh_stop").checked = 'checked';
}

function full_stop() {
  reset_buttons();
  send_command('stop');
  send_command('sh_stop');
}

window.onload = full_stop;
