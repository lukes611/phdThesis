var fs = require('fs');
var path = require('path');

var rem_last = function(s) { if(s[s.length-1] == "\n") return s.substring(0,s.length-1); return s; };

//calls cb_exists if directory d does exist, otherwise cb_does_not_exist
function directory_exists(d, cb_exists, cb_does_not_exist)
{
  fs.lstat(d, function(e, stats)
  {
    if(!e && stats.isDirectory())
    {
      if(cb_exists != undefined) cb_exists();
    }else
    {
        if(cb_does_not_exist != undefined) cb_does_not_exist();
    }
  });
}


//extract the data center name
var data_center_name = (fs.readFileSync('Pixel3DVideoInfo').toString().split('\n')[0]).replace("\n", "").replace("\r","");

if(process.argv.length == 3)
{

  var dir = data_center_name + '/' + process.argv[2];
  console.log('dir: ' + dir);

  directory_exists(dir, function()
  { //does exist, clear the data
    path.exists(dir + '/info', function(exists)
    {
      if(exists)
      {
        var numFiles = Number(fs.readFileSync(dir + '/info').toString().split('\n')[0]);
        console.log("removing [" + numFiles + '] frames');
        var i = 0;
        (function removeFileList(cb)
        {
          if(i < numFiles) { fs.unlinkSync(dir + '/f' + i); i++; removeFileList(cb); }
          else cb();
        })(function()
        {
          fs.writeFileSync(dir + '/info', '0');
        });

      }
      //console.log('does exist already');
    });

  },
  function()
  { //does not exist, create the directory
    fs.mkdir(dir, function(e)
    {
      if(e)
      {
        console.log(e);
      }
    })

  });

}else console.log('no name selected');
