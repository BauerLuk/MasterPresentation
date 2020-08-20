define(["require","exports","tslib","rondo/v1/index","rondo-saga/v1","rondo-modal-flows/v1/utility_modal_behavior"],(function(t,e,n,i,c,a){"use strict";Object.defineProperty(e,"__esModule",{value:!0}),e.ApproveBehavior=function(t){var e=t.listeners;return function(){this.initialState=function(){return{message:{}}},this.actionOpenApprove=i.createAction().handle((function(t,e){return{message:e}})),this.actionAccept=i.createAction().handle((function(t,e){return e})),this.actionCancel=i.createAction().handle((function(t,e){return e})),this.stringOrFunction=function(t,e){if(e)return"function"==typeof e?e(t):e},this.saga=i.createSaga((function(){var t,i,a;return n.__generator(this,(function(n){switch(n.label){case 0:return[4,c.raceListeners(e,"approveAction")];case 1:return(t=n.sent())?(i=t.action,a=t.listener,[4,c.put(this.actionOpenApprove)({title:this.stringOrFunction(i.payload,a.title),description:this.stringOrFunction(i.payload,a.description)})]):[3,0];case 2:return n.sent(),[4,c.race({accept:c.take([this.actionAccept]),close:c.take([this.actionCancel])})];case 3:return n.sent().accept?[4,c.put(a.nextAction)(i.payload)]:[3,6];case 4:return n.sent(),[4,c.put(this.actionCancel)()];case 5:n.sent(),n.label=6;case 6:return[3,0];case 7:return[2]}}))})),i.extendBehavior(this,a.UtilityModalBehavior({openAction:this.actionOpenApprove,actionAccept:this.actionAccept,closeAction:this.actionCancel}))}}}));
//# sourceMappingURL=approve_behavior.min.js-vflTmthcX.map