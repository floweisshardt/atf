function getJSON(folder, filename, callback) {
    $.getJSON(folder + filename + ".json")
        .done(function(json) {
            console.log("Request suceeded");
            writeDataToStorage(filename, json);
            if(callback) {
                callback();
            }
        })
        .fail(function(jqxhr, textStatus, error) {
            var err = textStatus + ", " + error;
            console.log("Request failed: " + err);
        });
}

function drawTestList() {
    var test_list = getDataFromStorage("test_list");
    document.getElementById("test_list").innerHTML = "";
    for (var number in test_list) {
        for(var test_name in test_list[number]) {
            document.getElementById("test_list").innerHTML += "<tr><td>" + (parseInt(number) + 1) + "</td><td>" + test_name + "</td><td>" + test_list[number][test_name].test_config + "</td><td><button type='button' class='btn btn-default' data-target='#test_detail' data-toggle='modal' data-name='" + test_name + "'>Details</button></td>";
        }
    }
}

function getTestDetails(test_name) {
    var test_detail = $('#test_detail');
    test_detail.find('.modal-title').html("Details " + test_name);
    test_detail.find('.modal-body p').html(JSON.stringify(sessionStorage.getItem(test_name)));
}

$(document).ready( function() {
    $('.btn-file :file').on('fileselect', function(event, numFiles, labels) {
        for (var x = 0; x < labels.length; x++) {
            if (labels[x] == "test_list") {
                getJSON("./data/", labels[x], drawTestList);
            } else {
                getJSON("./data/", labels[x]);
            }
        }
    });
    if (sessionStorage.getItem("test_list") != null) {
        drawTestList();
    }
});
