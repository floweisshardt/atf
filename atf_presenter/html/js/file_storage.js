function writeDataToStorage(name, data) {
    if (!name || !data) {
        return false;
    }

    if (typeof data === "object") {
        data = JSON.stringify(data);
    }
    sessionStorage.removeItem(name);
    sessionStorage.setItem(name, data);

    return true;
}

function getDataFromStorage(name) {
    var data = sessionStorage.getItem(name);

    if (!data) {
        return false;
    }

    // assume it is an object that has been stringified
    if (data[0] === "[" || data[0] === "{") {
        data = JSON.parse(data);
    }

    return data;
}

function clearStorage() {
    sessionStorage.clear();
}
