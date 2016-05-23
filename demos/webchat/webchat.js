#!/usr/bin/env node

var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});


io.on('connection', function(socket){
  socket.broadcast.emit('Hello snapcrafter');
  socket.on('chat message', function(user, msg){
    io.emit('chat message', user, msg);
  });
});

http.listen(3000, function(){
  console.log('listening on *:3000');
});
