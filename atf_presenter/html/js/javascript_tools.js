function getJSON(file) {
    $.getJSON(file)
        .done(function(json) {
            console.log("Request suceeded");
            console.log(json);
            drawTestList(json);
        })
        .fail(function(jqxhr, textStatus, error) {
            var err = textStatus + ", " + error;
            console.log("Request failed: " + err);
        });
}

$(document).ready(function() {
    var filename = "./data/test_list.json";
    getJSON(filename);
});

function drawTestList(test_list) {
    for(var number in test_list) {
        for(var test_name in test_list[number]) {
            document.getElementById("test_list").innerHTML += "<tr><td>" + (parseInt(number) + 1) + "</td><td>" + test_name + "</td><td>" + test_list[number][test_name].test_config + "</td><td><a class='btn btn-default' href='#' role='button' onclick='showTestDetails(" + '"' + test_name + '"' + ")'>Details</a></td>";
        }
    }
}