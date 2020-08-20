define("modules/clean/react/retrieval_success_banner/data/selectors",["require","exports","modules/clean/react/retrieval_success_banner/data/reducer","modules/clean/redux/selectors","modules/clean/redux/namespaces"],(function(e,r,n,s,t){"use strict";Object.defineProperty(r,"__esModule",{value:!0});var a=function(e){return s.getStateAtNamespace(e,t.RETRIEVAL_SUCCESS_BANNER_NAMESPACE_KEY)||n.defaultSuccessBannerState};r.isBrowseSuccessBannerCounterComplete=function(e){return a(e).browseSuccessBannerCounterComplete},r.isBrowseSuccessBannerVisible=function(e){return a(e).browseSuccessBannerShouldShow},r.isRetrievalSuccessBannerVisible=function(e){return a(e).retrievalSuccessBannerShouldShow},r.isSearchSuccessBannerVisible=function(e){return a(e).searchSuccessBannerShouldShow},r.isSearchBarAbandoned=function(e){return a(e).searchBarAbandoned},r.isSearchResultActionClicked=function(e){return a(e).searchResultActionClicked},r.isSearchSuccessBannerDismissed=function(e){return a(e).searchSuccessBannerDismissed},r.isInitializedByLocalStorage=function(e){return!!a(e).initializedByLocalStorage}})),define("modules/clean/react/retrieval_success_banner/data/action_creators",["require","exports","modules/clean/react/retrieval_success_banner/data/selectors","modules/clean/react/retrieval_success_banner/data/types","modules/clean/react/retrieval_success_banner/constants","modules/clean/storage"],(function(e,r,n,s,t,a){"use strict";function c(e){return{type:s.ActionTypes.UPDATE_RETRIEVAL_SUCCESS_BANNER_STATE,payload:{RetrievalSuccessBannerState:e}}}Object.defineProperty(r,"__esModule",{value:!0}),r.updateBrowseSuccessBannerCounterCompleteKey=function(e){return{type:s.ActionTypes.UPDATE_BSB_COUNTER_COMPLETE_KEY,payload:{browseSuccessBannerCounterComplete:e}}},r.updateRetrievalSuccessBannerState=c,r.updateRetrievalSuccessBannerVisibility=function(e){return{type:s.ActionTypes.UPDATE_RSB_VISIBILITY,payload:{retrievalSuccessBannerShouldShow:e}}},r.updateBrowseSuccessBannerVisibility=function(e){return{type:s.ActionTypes.UPDATE_BSB_VISIBILITY,payload:{browseSuccessBannerShouldShow:e}}},r.updateSearchSuccessBannerVisibility=function(e){return{type:s.ActionTypes.UPDATE_SSB_VISIBILITY,payload:{searchSuccessBannerShouldShow:e}}},r.updateSearchBarAbandoned=function(e){return{type:s.ActionTypes.UPDATE_SEARCH_BAR_ABANDONED,payload:{searchBarAbandoned:e}}},r.updateSearchResultBannerDismissed=function(e){return{type:s.ActionTypes.UPDATE_SEARCH_RESULT_BANNER_DISMISSED,payload:{searchSuccessBannerDismissed:e}}},r.updateSearchResultActionClicked=function(e){return{type:s.ActionTypes.UPDATE_SEARCH_RESULT_ACTION_CLICKED,payload:{searchResultActionClicked:e}}},r.initialStoreByLocalStorage=function(){return function(e,r){n.isInitializedByLocalStorage(r())||e(c({browseSuccessBannerCounterComplete:!!a.LocalStorage.get(t.BROWSE_SUCCESS_COUNT_DOWN_COMPLETE),browseSuccessBannerShouldShow:!!a.LocalStorage.get(t.BROWSE_SUCCESS_SHOW_BANNER_KEY),searchSuccessBannerShouldShow:!!a.LocalStorage.get(t.SEARCH_SUCCESS_SHOW_BANNER_KEY),initializedByLocalStorage:!0}))}}})),define("modules/clean/react/retrieval_success_banner/data/reducer",["require","exports","tslib","modules/clean/react/retrieval_success_banner/data/types"],(function(e,r,n,s){"use strict";Object.defineProperty(r,"__esModule",{value:!0}),r.defaultSuccessBannerState={browseSuccessBannerCounterComplete:!1,browseSuccessBannerShouldShow:!1,retrievalSuccessBannerShouldShow:!1,searchSuccessBannerShouldShow:!1,searchBarAbandoned:!1,searchSuccessBannerDismissed:!1,searchResultActionClicked:!1,initializedByLocalStorage:!1};r.successBannerReducer=function(e,t){switch(void 0===e&&(e=r.defaultSuccessBannerState),t.type){case s.ActionTypes.UPDATE_BSB_COUNTER_COMPLETE_KEY:case s.ActionTypes.UPDATE_BSB_VISIBILITY:case s.ActionTypes.UPDATE_RSB_VISIBILITY:case s.ActionTypes.UPDATE_SSB_VISIBILITY:case s.ActionTypes.UPDATE_SEARCH_BAR_ABANDONED:case s.ActionTypes.UPDATE_SEARCH_RESULT_BANNER_DISMISSED:case s.ActionTypes.UPDATE_SEARCH_RESULT_ACTION_CLICKED:return(function(e,r){return n.__assign(n.__assign({},e),r)})(e,t.payload);case s.ActionTypes.UPDATE_RETRIEVAL_SUCCESS_BANNER_STATE:return(function(e,r){return n.__assign(n.__assign({},e),r.RetrievalSuccessBannerState)})(e,t.payload);default:return e}}})),define("modules/clean/react/retrieval_success_banner/data/store",["require","exports","modules/clean/redux/namespaces","modules/clean/react/retrieval_success_banner/data/reducer","modules/clean/redux/store"],(function(e,r,n,s,t){"use strict";var a;Object.defineProperty(r,"__esModule",{value:!0}),r.getStoreForSuccessBanner=function(){var e;return a||(a=t.getStoreAndRegisterReducers(((e={})[n.RETRIEVAL_SUCCESS_BANNER_NAMESPACE_KEY]=s.successBannerReducer,e))),a}})),define("modules/clean/react/retrieval_success_banner/data/types",["require","exports"],(function(e,r){"use strict";Object.defineProperty(r,"__esModule",{value:!0}),(function(e){e.UPDATE_BSB_COUNTER_COMPLETE_KEY="UPDATE_BSB_COUNTER_COMPLETE_KEY",e.UPDATE_BSB_VISIBILITY="UPDATE_BSB_VISIBILITY",e.UPDATE_RSB_VISIBILITY="UPDATE_RSB_VISIBILITY",e.UPDATE_RETRIEVAL_SUCCESS_BANNER_STATE="UPDATE_RETRIEVAL_SUCCESS_BANNER_STATE",e.UPDATE_SSB_VISIBILITY="UPDATE_SSB_VISIBILITY",e.UPDATE_SEARCH_BAR_ABANDONED="UPDATE_SEARCH_BAR_ABANDONED",e.UPDATE_SEARCH_RESULT_BANNER_DISMISSED="UPDATE_SEARCH_RESULT_BANNER_DISMISSED",e.UPDATE_SEARCH_RESULT_ACTION_CLICKED="UPDATE_SEARCH_RESULT_ACTION_CLICKED"})(r.ActionTypes||(r.ActionTypes={}))}));var __importStar=this&&this.__importStar||function(e){if(e&&e.__esModule)return e;var r={};if(null!=e)for(var n in e)Object.hasOwnProperty.call(e,n)&&(r[n]=e[n]);return r.default=e,r};define("modules/clean/react/retrieval_success_banner/search_success_banner",["require","exports","modules/clean/react/async/loadable"],(function(e,r,n){"use strict";Object.defineProperty(r,"__esModule",{value:!0}),r.SearchSuccessBanner=n.Loadable({loader:function(){return new Promise((function(r,n){e(["modules/clean/react/retrieval_success_banner/search_success_component"],r,n)})).then(__importStar).then((function(e){return e.SearchSuccessBanner}))}})}));__importStar=this&&this.__importStar||function(e){if(e&&e.__esModule)return e;var r={};if(null!=e)for(var n in e)Object.hasOwnProperty.call(e,n)&&(r[n]=e[n]);return r.default=e,r};define("modules/clean/react/retrieval_success_banner/browse_success_banner",["require","exports","modules/clean/react/async/loadable"],(function(e,r,n){"use strict";Object.defineProperty(r,"__esModule",{value:!0}),r.BrowseSuccessBanner=n.Loadable({loader:function(){return new Promise((function(r,n){e(["modules/clean/react/retrieval_success_banner/browse_success_component"],r,n)})).then(__importStar).then((function(e){return e.BrowseSuccessBanner}))}})})),define("modules/clean/react/retrieval_success_banner/retrieval_success_browseview_banner",["require","exports","tslib","modules/clean/react/async/loadable","modules/clean/web_timing_logger"],(function(e,r,n,s,t){"use strict";Object.defineProperty(r,"__esModule",{value:!0}),r.RetrievalSuccessBrowseviewBanner=s.Loadable({loader:function(){return n.__awaiter(void 0,void 0,void 0,(function(){return n.__generator(this,(function(r){switch(r.label){case 0:return[4,t.waitForTTI()];case 1:return r.sent(),[4,new Promise((function(r,n){e(["modules/clean/react/retrieval_success_banner/retrieval_success_browseview_component"],r,n)})).then(n.__importStar)];case 2:return[2,r.sent().RetrievalSuccessBrowseviewBannerWithProvider]}}))}))}})})),define("modules/clean/react/retrieval_success_banner/retrieval_success_home_banner",["require","exports","tslib","modules/clean/react/async/loadable","modules/clean/web_timing_logger"],(function(e,r,n,s,t){"use strict";Object.defineProperty(r,"__esModule",{value:!0}),r.RetrievalSuccessHomeBanner=s.Loadable({displayName:"RetrievalSuccessHomeBanner",loader:function(){return n.__awaiter(void 0,void 0,void 0,(function(){return n.__generator(this,(function(r){switch(r.label){case 0:return[4,t.waitForTTI()];case 1:return r.sent(),[4,new Promise((function(r,n){e(["modules/clean/react/retrieval_success_banner/retrieval_success_home_component"],r,n)})).then(n.__importStar)];case 2:return[2,r.sent().RetrievalSuccessHomeBannerWithProvider]}}))}))}})})),define("modules/clean/react/retrieval_success_banner/retrieval_success_filesview_banner",["require","exports","tslib","modules/clean/react/async/loadable","modules/clean/web_timing_logger"],(function(e,r,n,s,t){"use strict";Object.defineProperty(r,"__esModule",{value:!0}),r.RetrievalSuccessFilesviewBanner=s.Loadable({loader:function(){return n.__awaiter(void 0,void 0,void 0,(function(){return n.__generator(this,(function(r){switch(r.label){case 0:return[4,t.waitForTTI()];case 1:return r.sent(),[4,new Promise((function(r,n){e(["modules/clean/react/retrieval_success_banner/retrieval_success_filesview_component"],r,n)})).then(n.__importStar)];case 2:return[2,r.sent().RetrievalSuccessFilesviewBannerWithProvider]}}))}))}})}));
//# sourceMappingURL=pkg-retrieval-success-banner.min.js-vflpcTpit.map