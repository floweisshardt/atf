var emergency_storage = {};
function writeDataToStorage(name, data) {
  if (!name || !data) {
    return false;
  }

  if (typeof data === 'object') {
    data = JSON.stringify(data);
  }
  sessionStorage.removeItem(name);
  try {
    sessionStorage.setItem(name, data);
  } catch (error) {
    console.log(error);
    console.log("SessionStorage at full capacity! Switching to emergency storage!");
    delete emergency_storage[name];
    emergency_storage[name] = data;
  }

  return true;
}

function getDataFromStorage(name) {
  var data = sessionStorage.getItem(name);

  if (!data) {
    if (!emergency_storage.hasOwnProperty(name)) {
      return false;
    } else {
      data = emergency_storage[name];
    }
  }

  // assume it is an object that has been stringified
  if (data[0] === '[' || data[0] === '{') {
    data = JSON.parse(data);
  }

  return data;
}

function clearStorage() {
  sessionStorage.clear();
}

function removeDataFromStorage(name) {
  sessionStorage.removeItem(name);
}
