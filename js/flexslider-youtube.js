// Youtube and Vimeo Flexslider Control
// Partly from Duppi on GitHub, partly from digging into the YouTube API
// Need to set this up to use classes instead of IDs
// See: https://github.com/woothemes/FlexSlider/issues/346#issuecomment-20342848

var tag = document.createElement('script');
tag.src = "http://www.youtube.com/player_api";
var firstScriptTag = document.getElementsByTagName('script')[0];
firstScriptTag.parentNode.insertBefore(tag, firstScriptTag);

jQuery(window).load(function() {
    var vimeoPlayers = jQuery('#hero .flexslider').find('iframe'), player;

    for (var i = 0, length = vimeoPlayers.length; i < length; i++) {
        player = vimeoPlayers[i];
        $f(player).addEvent('ready', ready);
    }

    function addEvent(element, eventName, callback) {
        if (element.addEventListener) {
            element.addEventListener(eventName, callback, false)
        } else {
            element.attachEvent(eventName, callback, false);
        }
    }

    function ready(player_id) {
        var froogaloop = $f(player_id);
        froogaloop.addEvent('play', function(data) {
            jQuery('#hero .flexslider').flexslider("pause");
        });

        froogaloop.addEvent('pause', function(data) {
            jQuery('#hero .flexslider').flexslider("play");
        });
    }

    jQuery("#hero .flexslider")
    .flexslider({
        before: function(slider){
            if (slider.slides.eq(slider.currentSlide).find('iframe').length !== 0)
                $f( slider.slides.eq(slider.currentSlide).find('iframe').attr('id') ).api('pause');
            /* ------------------  YOUTUBE FOR AUTOSLIDER ------------------ */
            playVideoAndPauseOthers($('.play3 iframe')[0]);
        }
    });

    function playVideoAndPauseOthers(frame) {
        jQuery('iframe').each(function(i) {
            var func = this === frame ? 'playVideo' : 'stopVideo';
            this.contentWindow.postMessage('{"event":"command","func":"' + func + '","args":""}', '*');
        });
    }

    /* ------------ PREV & NEXT BUTTON FOR FLEXSLIDER (YOUTUBE) ------------ */

    jQuery('.flex-next, .flex-prev').click(function() {
        playVideoAndPauseOthers($('.play3 iframe')[0]);
    });


    //YOUTUBE STUFF

    function controlSlider(event) {
        console.log(event);
        var playerstate=event.data;
        console.log(playerstate);
        if(playerstate==1 || playerstate==3){
            jQuery('#hero .flexslider').flexslider("pause");
        };
        if(playerstate==0 || playerstate==2){
            jQuery('#hero .flexslider').flexslider("play");
        };
    };

    var player;
    function onYouTubeIframeAPIReady() {
        player = new YT.Player('youtubevideo', {
            playerVars: { 'controls': 0, 'modestbranding': 0, 'showinfo': 0 },
            events: {
                'onStateChange': onPlayerStateChange
            }
        });
    }
});
