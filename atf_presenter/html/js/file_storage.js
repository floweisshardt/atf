var FileStorage = {
  name: '',
  data: {},
  emergencyStorage: {},
  writeData: function () {
    if (!this.name || !this.data) {
      return false;
    }

    if (typeof this.data === 'object') {
      this.data = JSON.stringify(this.data);
    }
    sessionStorage.removeItem(this.name);
    try {
      sessionStorage.setItem(this.name, this.data);
    } catch (error) {
      console.log(error);
      console.log("SessionStorage at full capacity! Switching to emergency storage!");
      delete this.emergencyStorage[this.name];
      this.emergencyStorage[this.name] = this.data;
    }
    return true;
  },
  readData: function () {
    var data = sessionStorage.getItem(this.name);

    if (!data) {
      if (!this.emergencyStorage.hasOwnProperty(this.name)) {
        return false;
      } else {
        data = this.emergencyStorage[this.name];
      }
    }

    // Assume it is an object that has been stringified
    if (data[0] === '[' || data[0] === '{') {
      data = JSON.parse(data);
    }

    return data;
  },
  removeData: function() {
    sessionStorage.removeItem(this.name);
  },
  clear: function () {
    sessionStorage.clear();
  }
};
