$(".nav-sidebar li, .detail_button").each(function() {
    $(this).on("click", function() {
        $(".content_container").load($(this).attr("data-page"));
    });
});

$("#detail_button").on("click", function(e) {
        e.stopImmediatePropagation();
        alert("click");
        $(".content_container").load($(this).attr("data-page"));

});

$('.sidebar > ul.nav li a').click(function(e) {
    $('.sr-only').remove();
    var $this = $(this);
    $this.parent().siblings().removeClass('active').end().addClass('active');
    $this.append('<span class="sr-only">(current)</span>');
    e.preventDefault();
});

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
