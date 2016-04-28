Number.prototype.round = function (places) {
  return +(Math.round(this + 'e+' + places)  + 'e-' + places);
};

String.prototype.capitalize = function () {
  return this[0].toUpperCase() + this.substring(1);
};

function handleFileSelect(evt) {
  FileStorage.clear();

  var files = evt.target.files; // FileList object
  var file_list = {};
  var progressbar = $('#file_upload_progressbar');
  var files_read = 0;

  progressbar.empty();
  progressbar.css('width', '0%').attr('aria-valuenow', '0%');
  progressbar.append('0%');

  // Loop through the FileList
  for (var i = 0, f; f = files[i]; i++) {

    // Only process json files.
    if (f.name.indexOf('.json') === -1) {
      continue;
    }

    var reader = new FileReader();

    reader.onload = (function (file) {
      return function (e) {
        files_read++;
        var percentLoaded = Math.round((files_read / files.length) * 100);

        file_list[file.name] = JSON.parse(e.target.result);

        progressbar.empty();
        progressbar.css('width', percentLoaded + '%').attr('aria-valuenow', percentLoaded);
        progressbar.append(percentLoaded + '%');

        if (file.name.indexOf('test_list') != -1) {
          file_list[file.name] = TestList.convert(file_list[file.name]);
        }
        if (!FileStorage.writeData(file.name.split('.')[0], file_list[file.name])) {
          console.log('Writing to storage failed!');
        } else {
          console.log('Request succeeded');
        }

        if (Object.keys(file_list).length === files.length) {
          if (!(file_list.hasOwnProperty('test_list.json'))) {
            bootbox.alert('You have to select the test_list.json file!');
          } else {
            TestList.summarize();
            TestList.show();
          }
        }
      };
    })(f);
    reader.readAsText(f);
  }
}

function handleFileSelectDropbox() {
  FileStorage.clear();

  var file_list = {};
  var progressbar = $('#file_upload_progressbar');
  var files_read = 0;

  progressbar.empty();
  progressbar.css('width', '0%').attr('aria-valuenow', '0%');
  progressbar.append('0%');

  var options = {
    success: function(files) {
      $.each(files, function (index, file) {
        $.get(file.link + '?callback', 'jsonp',  function (data) {
          files_read++;
          var percentLoaded = Math.round((files_read / files.length) * 100);
          file_list[file.name] = JSON.parse(data);

          progressbar.empty();
          progressbar.css('width', percentLoaded + '%').attr('aria-valuenow', percentLoaded);
          progressbar.append(percentLoaded + '%');

          if (file.name.indexOf('test_list') != -1) {
            file_list[file.name] = TestList.convert(file_list[file.name]);
          }

          if (!FileStorage.writeData(file.name.split('.')[0], file_list[file.name])) {
            console.log('Writing to storage failed!');
          } else {
            console.log('Request succeeded');
          }

          if (Object.keys(file_list).length === files.length) {
            if (!(file_list.hasOwnProperty('test_list.json'))) {
              bootbox.alert('You have to select the test_list.json file!');
            } else {
              TestList.summarize();
              TestList.show();
            }
          }
        });


      });
    },
    cancel: function() {

    },
    linkType: "direct",
    multiselect: true,
    extensions: ['.json']
  };

  Dropbox.choose(options);
}
