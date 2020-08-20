define(["require","exports","tslib","react","react-intl","spectrum/icon_content","spectrum/facepile/facepile_members","modules/core/i18n","modules/clean/static_urls"],(function(e,t,a,l,s,n,r,i,o){"use strict";Object.defineProperty(t,"__esModule",{value:!0}),l=a.__importDefault(l),t.InviteModalSidebar=function(e){var t=e.baseClass,c=e.memberList,m=e.emails,u=e.sentEmails,d=l.default.createElement(l.default.Fragment,null,i.intl.formatMessage({id:"b21B7N",defaultMessage:"Anyone you invite will have access to team folders"})),f=i.intl.formatMessage({id:"DZeWex",defaultMessage:"You can change access to any folder at any time."});if(u.length&&(d=l.default.createElement(l.default.Fragment,null,i.intl.formatMessage({id:"RMQzK3",defaultMessage:"Invites sent. Everyone that accepts can access team folders."}))),m.length){var g=a.__spreadArrays([i.intl.formatMessage({id:"1oYZCT",defaultMessage:"You"})],m.slice(0,2)),h=c.length-1;g.length<3&&(g=g.concat(c.slice(1,2).map((function(e){return e.email})))),m.length+h>=3&&g.splice(2,1,i.intl.formatMessage({id:"fkU/W8",defaultMessage:"{amount, plural, one{# other} other{# others}}"},{amount:m.length+h-1})),d=l.default.createElement(l.default.Fragment,null,i.intl.formatMessage({id:"2ohcMK",defaultMessage:"{UsersWithAccess} will have access to team folders."},{UsersWithAccess:l.default.createElement(s.FormattedList,{type:"conjunction",value:g})}))}var p,_=m.map((function(e){return{initials:e[0].toUpperCase(),name:e,photo:null}})),v=c.map((function(e){return{initials:e.initials,name:e.display_name,photo:e.photo_url}})),M=v[0],b=v.slice(1),E=a.__spreadArrays([M],_,b);return E.length>3&&(p=(E.length-3+1).toString(),E.length=2),l.default.createElement("div",{className:t+"__sidebar"},l.default.createElement("div",{className:t+"__sidebar-image"},l.default.createElement(n.IconContent,{name:"folder_team-large"}),E.length>1&&l.default.createElement(r.FacepileMembers,{remainderCount:p,shadowBackgroundColor:"#0d2484",avatarSize:32,members:E.map((function(e,t){return{memberKey:"PhotoActive-"+t,initials:e.initials,photoUrl:M.name===e.name?M.photo:o.static_url("/static/images/growth/member-vflJ3QUg7.svg"),alt:null,active:!0,tooltipContent:null,avatarColorSeed:e.name}}))})),l.default.createElement("div",{className:t+"__sidebar-title"},d),l.default.createElement("div",{className:t+"__sidebar-subtitle"},f))}}));
//# sourceMappingURL=invite_modal_sidebar.min.js-vflDfFpPw.map