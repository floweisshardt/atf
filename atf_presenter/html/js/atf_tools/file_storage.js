var FileStorage = {
  emergencyStorage: {},
  writeData: function (name, data) {
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
      delete this.emergencyStorage[name];
      this.emergencyStorage[name] = data;
    }
    return true;
  },
  readData: function (name) {
    var data = sessionStorage.getItem(name);

    if (!data) {
      if (!this.emergencyStorage.hasOwnProperty(name)) {
        return false;
      } else {
        data = this.emergencyStorage[name];
      }
    }

    // Assume it is an object that has been stringified
    if (data[0] === '[' || data[0] === '{') {
      data = JSON.parse(data);
    }

    return data;
  },
  removeData: function(name) {
    sessionStorage.removeItem(name);
  },
  clear: function () {
    sessionStorage.clear();
  }
};
