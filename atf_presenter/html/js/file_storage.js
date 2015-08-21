function writeDataToStorage(name, data) {
    if (!name || !data) {
        return;
    }

    if (typeof data === "object") {
        data = JSON.stringify(data);
    }
    sessionStorage.setItem(name, data);
}

function getDataFromStorage(name) {
    var data = sessionStorage.getItem(name);

    if (!data) {
        return;
    }

    // assume it is an object that has been stringified
    if (data[0] === "[" && data[1] === "{") {
        data = JSON.parse(data);
    }

    return data;
}

$(document).on('change', '.btn-file :file', function() {
    var labels = [];
    var input = $(this),
        numFiles = input.get(0).files ? input.get(0).files.length : 1;
    for (var files = 0; files < input.get(0).files.length; files++) {
        labels.push(input.get(0).files[parseInt(files)].name.split(".")[0]);
    }
    if ($.inArray("test_list", labels) === -1) {
        alert("You have to select the test_list.yaml file!");
    }
    input.trigger('fileselect', [numFiles, labels]);
});
