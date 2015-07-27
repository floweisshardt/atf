function getJSON() {
    $.getJSON("js/ts1_t1.json")
        .done(function( json ) {
        console.log(json);
        })
        .fail(function( jqxhr, textStatus, error ) {
            var err = textStatus + ", " + error;
            console.log("Request Failed: " + err);
        });
}