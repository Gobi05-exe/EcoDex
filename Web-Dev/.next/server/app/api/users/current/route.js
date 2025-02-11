/*
 * ATTENTION: An "eval-source-map" devtool has been used.
 * This devtool is neither made for production nor for readable output files.
 * It uses "eval()" calls to create a separate source file with attached SourceMaps in the browser devtools.
 * If you are trying to read the output file, select a different devtool (https://webpack.js.org/configuration/devtool/)
 * or disable the default devtool with "devtool: false".
 * If you are looking for production-ready output files, see mode: "production" (https://webpack.js.org/configuration/mode/).
 */
(() => {
var exports = {};
exports.id = "app/api/users/current/route";
exports.ids = ["app/api/users/current/route"];
exports.modules = {

/***/ "mongodb":
/*!**************************!*\
  !*** external "mongodb" ***!
  \**************************/
/***/ ((module) => {

"use strict";
module.exports = require("mongodb");

/***/ }),

/***/ "next/dist/compiled/next-server/app-page.runtime.dev.js":
/*!*************************************************************************!*\
  !*** external "next/dist/compiled/next-server/app-page.runtime.dev.js" ***!
  \*************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/compiled/next-server/app-page.runtime.dev.js");

/***/ }),

/***/ "next/dist/compiled/next-server/app-route.runtime.dev.js":
/*!**************************************************************************!*\
  !*** external "next/dist/compiled/next-server/app-route.runtime.dev.js" ***!
  \**************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/compiled/next-server/app-route.runtime.dev.js");

/***/ }),

/***/ "../app-render/after-task-async-storage.external":
/*!***********************************************************************************!*\
  !*** external "next/dist/server/app-render/after-task-async-storage.external.js" ***!
  \***********************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/server/app-render/after-task-async-storage.external.js");

/***/ }),

/***/ "../app-render/work-async-storage.external":
/*!*****************************************************************************!*\
  !*** external "next/dist/server/app-render/work-async-storage.external.js" ***!
  \*****************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/server/app-render/work-async-storage.external.js");

/***/ }),

/***/ "./work-unit-async-storage.external":
/*!**********************************************************************************!*\
  !*** external "next/dist/server/app-render/work-unit-async-storage.external.js" ***!
  \**********************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/server/app-render/work-unit-async-storage.external.js");

/***/ }),

/***/ "(rsc)/./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fusers%2Fcurrent%2Froute&page=%2Fapi%2Fusers%2Fcurrent%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fusers%2Fcurrent%2Froute.ts&appDir=C%3A%5CUsers%5CArjun%5CDesktop%5CBankEase%20Copies%5CEco-Dex%5CWebDev%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=C%3A%5CUsers%5CArjun%5CDesktop%5CBankEase%20Copies%5CEco-Dex%5CWebDev&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D!":
/*!*************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************!*\
  !*** ./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fusers%2Fcurrent%2Froute&page=%2Fapi%2Fusers%2Fcurrent%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fusers%2Fcurrent%2Froute.ts&appDir=C%3A%5CUsers%5CArjun%5CDesktop%5CBankEase%20Copies%5CEco-Dex%5CWebDev%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=C%3A%5CUsers%5CArjun%5CDesktop%5CBankEase%20Copies%5CEco-Dex%5CWebDev&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D! ***!
  \*************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   patchFetch: () => (/* binding */ patchFetch),\n/* harmony export */   routeModule: () => (/* binding */ routeModule),\n/* harmony export */   serverHooks: () => (/* binding */ serverHooks),\n/* harmony export */   workAsyncStorage: () => (/* binding */ workAsyncStorage),\n/* harmony export */   workUnitAsyncStorage: () => (/* binding */ workUnitAsyncStorage)\n/* harmony export */ });\n/* harmony import */ var next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! next/dist/server/route-modules/app-route/module.compiled */ \"(rsc)/./node_modules/next/dist/server/route-modules/app-route/module.compiled.js\");\n/* harmony import */ var next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0___default = /*#__PURE__*/__webpack_require__.n(next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0__);\n/* harmony import */ var next_dist_server_route_kind__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! next/dist/server/route-kind */ \"(rsc)/./node_modules/next/dist/server/route-kind.js\");\n/* harmony import */ var next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! next/dist/server/lib/patch-fetch */ \"(rsc)/./node_modules/next/dist/server/lib/patch-fetch.js\");\n/* harmony import */ var next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2___default = /*#__PURE__*/__webpack_require__.n(next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2__);\n/* harmony import */ var C_Users_Arjun_Desktop_BankEase_Copies_Eco_Dex_WebDev_app_api_users_current_route_ts__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./app/api/users/current/route.ts */ \"(rsc)/./app/api/users/current/route.ts\");\n\n\n\n\n// We inject the nextConfigOutput here so that we can use them in the route\n// module.\nconst nextConfigOutput = \"\"\nconst routeModule = new next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0__.AppRouteRouteModule({\n    definition: {\n        kind: next_dist_server_route_kind__WEBPACK_IMPORTED_MODULE_1__.RouteKind.APP_ROUTE,\n        page: \"/api/users/current/route\",\n        pathname: \"/api/users/current\",\n        filename: \"route\",\n        bundlePath: \"app/api/users/current/route\"\n    },\n    resolvedPagePath: \"C:\\\\Users\\\\Arjun\\\\Desktop\\\\BankEase Copies\\\\Eco-Dex\\\\WebDev\\\\app\\\\api\\\\users\\\\current\\\\route.ts\",\n    nextConfigOutput,\n    userland: C_Users_Arjun_Desktop_BankEase_Copies_Eco_Dex_WebDev_app_api_users_current_route_ts__WEBPACK_IMPORTED_MODULE_3__\n});\n// Pull out the exports that we need to expose from the module. This should\n// be eliminated when we've moved the other routes to the new format. These\n// are used to hook into the route.\nconst { workAsyncStorage, workUnitAsyncStorage, serverHooks } = routeModule;\nfunction patchFetch() {\n    return (0,next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2__.patchFetch)({\n        workAsyncStorage,\n        workUnitAsyncStorage\n    });\n}\n\n\n//# sourceMappingURL=app-route.js.map//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHJzYykvLi9ub2RlX21vZHVsZXMvbmV4dC9kaXN0L2J1aWxkL3dlYnBhY2svbG9hZGVycy9uZXh0LWFwcC1sb2FkZXIvaW5kZXguanM/bmFtZT1hcHAlMkZhcGklMkZ1c2VycyUyRmN1cnJlbnQlMkZyb3V0ZSZwYWdlPSUyRmFwaSUyRnVzZXJzJTJGY3VycmVudCUyRnJvdXRlJmFwcFBhdGhzPSZwYWdlUGF0aD1wcml2YXRlLW5leHQtYXBwLWRpciUyRmFwaSUyRnVzZXJzJTJGY3VycmVudCUyRnJvdXRlLnRzJmFwcERpcj1DJTNBJTVDVXNlcnMlNUNBcmp1biU1Q0Rlc2t0b3AlNUNCYW5rRWFzZSUyMENvcGllcyU1Q0Vjby1EZXglNUNXZWJEZXYlNUNhcHAmcGFnZUV4dGVuc2lvbnM9dHN4JnBhZ2VFeHRlbnNpb25zPXRzJnBhZ2VFeHRlbnNpb25zPWpzeCZwYWdlRXh0ZW5zaW9ucz1qcyZyb290RGlyPUMlM0ElNUNVc2VycyU1Q0FyanVuJTVDRGVza3RvcCU1Q0JhbmtFYXNlJTIwQ29waWVzJTVDRWNvLURleCU1Q1dlYkRldiZpc0Rldj10cnVlJnRzY29uZmlnUGF0aD10c2NvbmZpZy5qc29uJmJhc2VQYXRoPSZhc3NldFByZWZpeD0mbmV4dENvbmZpZ091dHB1dD0mcHJlZmVycmVkUmVnaW9uPSZtaWRkbGV3YXJlQ29uZmlnPWUzMCUzRCEiLCJtYXBwaW5ncyI6Ijs7Ozs7Ozs7Ozs7Ozs7QUFBK0Y7QUFDdkM7QUFDcUI7QUFDK0M7QUFDNUg7QUFDQTtBQUNBO0FBQ0Esd0JBQXdCLHlHQUFtQjtBQUMzQztBQUNBLGNBQWMsa0VBQVM7QUFDdkI7QUFDQTtBQUNBO0FBQ0E7QUFDQSxLQUFLO0FBQ0w7QUFDQTtBQUNBLFlBQVk7QUFDWixDQUFDO0FBQ0Q7QUFDQTtBQUNBO0FBQ0EsUUFBUSxzREFBc0Q7QUFDOUQ7QUFDQSxXQUFXLDRFQUFXO0FBQ3RCO0FBQ0E7QUFDQSxLQUFLO0FBQ0w7QUFDMEY7O0FBRTFGIiwic291cmNlcyI6WyIiXSwic291cmNlc0NvbnRlbnQiOlsiaW1wb3J0IHsgQXBwUm91dGVSb3V0ZU1vZHVsZSB9IGZyb20gXCJuZXh0L2Rpc3Qvc2VydmVyL3JvdXRlLW1vZHVsZXMvYXBwLXJvdXRlL21vZHVsZS5jb21waWxlZFwiO1xuaW1wb3J0IHsgUm91dGVLaW5kIH0gZnJvbSBcIm5leHQvZGlzdC9zZXJ2ZXIvcm91dGUta2luZFwiO1xuaW1wb3J0IHsgcGF0Y2hGZXRjaCBhcyBfcGF0Y2hGZXRjaCB9IGZyb20gXCJuZXh0L2Rpc3Qvc2VydmVyL2xpYi9wYXRjaC1mZXRjaFwiO1xuaW1wb3J0ICogYXMgdXNlcmxhbmQgZnJvbSBcIkM6XFxcXFVzZXJzXFxcXEFyanVuXFxcXERlc2t0b3BcXFxcQmFua0Vhc2UgQ29waWVzXFxcXEVjby1EZXhcXFxcV2ViRGV2XFxcXGFwcFxcXFxhcGlcXFxcdXNlcnNcXFxcY3VycmVudFxcXFxyb3V0ZS50c1wiO1xuLy8gV2UgaW5qZWN0IHRoZSBuZXh0Q29uZmlnT3V0cHV0IGhlcmUgc28gdGhhdCB3ZSBjYW4gdXNlIHRoZW0gaW4gdGhlIHJvdXRlXG4vLyBtb2R1bGUuXG5jb25zdCBuZXh0Q29uZmlnT3V0cHV0ID0gXCJcIlxuY29uc3Qgcm91dGVNb2R1bGUgPSBuZXcgQXBwUm91dGVSb3V0ZU1vZHVsZSh7XG4gICAgZGVmaW5pdGlvbjoge1xuICAgICAgICBraW5kOiBSb3V0ZUtpbmQuQVBQX1JPVVRFLFxuICAgICAgICBwYWdlOiBcIi9hcGkvdXNlcnMvY3VycmVudC9yb3V0ZVwiLFxuICAgICAgICBwYXRobmFtZTogXCIvYXBpL3VzZXJzL2N1cnJlbnRcIixcbiAgICAgICAgZmlsZW5hbWU6IFwicm91dGVcIixcbiAgICAgICAgYnVuZGxlUGF0aDogXCJhcHAvYXBpL3VzZXJzL2N1cnJlbnQvcm91dGVcIlxuICAgIH0sXG4gICAgcmVzb2x2ZWRQYWdlUGF0aDogXCJDOlxcXFxVc2Vyc1xcXFxBcmp1blxcXFxEZXNrdG9wXFxcXEJhbmtFYXNlIENvcGllc1xcXFxFY28tRGV4XFxcXFdlYkRldlxcXFxhcHBcXFxcYXBpXFxcXHVzZXJzXFxcXGN1cnJlbnRcXFxccm91dGUudHNcIixcbiAgICBuZXh0Q29uZmlnT3V0cHV0LFxuICAgIHVzZXJsYW5kXG59KTtcbi8vIFB1bGwgb3V0IHRoZSBleHBvcnRzIHRoYXQgd2UgbmVlZCB0byBleHBvc2UgZnJvbSB0aGUgbW9kdWxlLiBUaGlzIHNob3VsZFxuLy8gYmUgZWxpbWluYXRlZCB3aGVuIHdlJ3ZlIG1vdmVkIHRoZSBvdGhlciByb3V0ZXMgdG8gdGhlIG5ldyBmb3JtYXQuIFRoZXNlXG4vLyBhcmUgdXNlZCB0byBob29rIGludG8gdGhlIHJvdXRlLlxuY29uc3QgeyB3b3JrQXN5bmNTdG9yYWdlLCB3b3JrVW5pdEFzeW5jU3RvcmFnZSwgc2VydmVySG9va3MgfSA9IHJvdXRlTW9kdWxlO1xuZnVuY3Rpb24gcGF0Y2hGZXRjaCgpIHtcbiAgICByZXR1cm4gX3BhdGNoRmV0Y2goe1xuICAgICAgICB3b3JrQXN5bmNTdG9yYWdlLFxuICAgICAgICB3b3JrVW5pdEFzeW5jU3RvcmFnZVxuICAgIH0pO1xufVxuZXhwb3J0IHsgcm91dGVNb2R1bGUsIHdvcmtBc3luY1N0b3JhZ2UsIHdvcmtVbml0QXN5bmNTdG9yYWdlLCBzZXJ2ZXJIb29rcywgcGF0Y2hGZXRjaCwgIH07XG5cbi8vIyBzb3VyY2VNYXBwaW5nVVJMPWFwcC1yb3V0ZS5qcy5tYXAiXSwibmFtZXMiOltdLCJpZ25vcmVMaXN0IjpbXSwic291cmNlUm9vdCI6IiJ9\n//# sourceURL=webpack-internal:///(rsc)/./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fusers%2Fcurrent%2Froute&page=%2Fapi%2Fusers%2Fcurrent%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fusers%2Fcurrent%2Froute.ts&appDir=C%3A%5CUsers%5CArjun%5CDesktop%5CBankEase%20Copies%5CEco-Dex%5CWebDev%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=C%3A%5CUsers%5CArjun%5CDesktop%5CBankEase%20Copies%5CEco-Dex%5CWebDev&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D!\n");

/***/ }),

/***/ "(rsc)/./node_modules/next/dist/build/webpack/loaders/next-flight-client-entry-loader.js?server=true!":
/*!******************************************************************************************************!*\
  !*** ./node_modules/next/dist/build/webpack/loaders/next-flight-client-entry-loader.js?server=true! ***!
  \******************************************************************************************************/
/***/ (() => {



/***/ }),

/***/ "(ssr)/./node_modules/next/dist/build/webpack/loaders/next-flight-client-entry-loader.js?server=true!":
/*!******************************************************************************************************!*\
  !*** ./node_modules/next/dist/build/webpack/loaders/next-flight-client-entry-loader.js?server=true! ***!
  \******************************************************************************************************/
/***/ (() => {



/***/ }),

/***/ "(rsc)/./app/api/users/current/route.ts":
/*!****************************************!*\
  !*** ./app/api/users/current/route.ts ***!
  \****************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   GET: () => (/* binding */ GET)\n/* harmony export */ });\n/* harmony import */ var next_server__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! next/server */ \"(rsc)/./node_modules/next/dist/api/server.js\");\n/* harmony import */ var _utils_mongodb__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @/utils/mongodb */ \"(rsc)/./utils/mongodb.ts\");\n// /app/api/user/current/route.ts\n\n\n// Importing cookie/session library (for example, if you're using next-auth or a similar library)\n// You can use Redis if you set up session storage with Redis\nasync function GET(request) {\n    try {\n        // Assuming you have stored the username in a cookie or session\n        const cookie = request.cookies.get('username'); // Fetch username from cookie\n        if (!cookie) {\n            return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n                error: 'Username not found'\n            }, {\n                status: 401\n            });\n        }\n        const username = cookie.value; // Extract username value from the cookie\n        // Connect to MongoDB and retrieve documents\n        const { db } = await (0,_utils_mongodb__WEBPACK_IMPORTED_MODULE_1__.connectToDatabase)();\n        const collectionName = `${username}_waste_records`;\n        console.log(\"Collection name:\", collectionName); // Debugging line\n        const collection = db.collection(collectionName);\n        const documents = await collection.find({}).toArray();\n        return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json(documents);\n    } catch (error) {\n        console.error('Error fetching records:', error);\n        return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n            error: 'Failed to fetch records'\n        }, {\n            status: 500\n        });\n    }\n}\n//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHJzYykvLi9hcHAvYXBpL3VzZXJzL2N1cnJlbnQvcm91dGUudHMiLCJtYXBwaW5ncyI6Ijs7Ozs7O0FBQUEsaUNBQWlDO0FBQ3VCO0FBQ0o7QUFDcEQsaUdBQWlHO0FBQ2pHLDZEQUE2RDtBQUV0RCxlQUFlRSxJQUFJQyxPQUFvQjtJQUMxQyxJQUFJO1FBQ0EsK0RBQStEO1FBQy9ELE1BQU1DLFNBQVNELFFBQVFFLE9BQU8sQ0FBQ0MsR0FBRyxDQUFDLGFBQWEsNkJBQTZCO1FBRTdFLElBQUksQ0FBQ0YsUUFBUTtZQUNULE9BQU9KLHFEQUFZQSxDQUFDTyxJQUFJLENBQUM7Z0JBQUVDLE9BQU87WUFBcUIsR0FBRztnQkFBRUMsUUFBUTtZQUFJO1FBQzVFO1FBRUEsTUFBTUMsV0FBV04sT0FBT08sS0FBSyxFQUFFLHlDQUF5QztRQUV4RSw0Q0FBNEM7UUFDNUMsTUFBTSxFQUFFQyxFQUFFLEVBQUUsR0FBRyxNQUFNWCxpRUFBaUJBO1FBQ3RDLE1BQU1ZLGlCQUFpQixHQUFHSCxTQUFTLGNBQWMsQ0FBQztRQUNsREksUUFBUUMsR0FBRyxDQUFDLG9CQUFvQkYsaUJBQWlCLGlCQUFpQjtRQUVsRSxNQUFNRyxhQUFhSixHQUFHSSxVQUFVLENBQUNIO1FBQ2pDLE1BQU1JLFlBQVksTUFBTUQsV0FBV0UsSUFBSSxDQUFDLENBQUMsR0FBR0MsT0FBTztRQUVuRCxPQUFPbkIscURBQVlBLENBQUNPLElBQUksQ0FBQ1U7SUFDN0IsRUFBRSxPQUFPVCxPQUFPO1FBQ1pNLFFBQVFOLEtBQUssQ0FBQywyQkFBMkJBO1FBQ3pDLE9BQU9SLHFEQUFZQSxDQUFDTyxJQUFJLENBQUM7WUFBRUMsT0FBTztRQUEwQixHQUFHO1lBQUVDLFFBQVE7UUFBSTtJQUNqRjtBQUNKIiwic291cmNlcyI6WyJDOlxcVXNlcnNcXEFyanVuXFxEZXNrdG9wXFxCYW5rRWFzZSBDb3BpZXNcXEVjby1EZXhcXFdlYkRldlxcYXBwXFxhcGlcXHVzZXJzXFxjdXJyZW50XFxyb3V0ZS50cyJdLCJzb3VyY2VzQ29udGVudCI6WyIvLyAvYXBwL2FwaS91c2VyL2N1cnJlbnQvcm91dGUudHNcclxuaW1wb3J0IHsgTmV4dFJlcXVlc3QsIE5leHRSZXNwb25zZSB9IGZyb20gJ25leHQvc2VydmVyJztcclxuaW1wb3J0IHsgY29ubmVjdFRvRGF0YWJhc2UgfSBmcm9tICdAL3V0aWxzL21vbmdvZGInO1xyXG4vLyBJbXBvcnRpbmcgY29va2llL3Nlc3Npb24gbGlicmFyeSAoZm9yIGV4YW1wbGUsIGlmIHlvdSdyZSB1c2luZyBuZXh0LWF1dGggb3IgYSBzaW1pbGFyIGxpYnJhcnkpXHJcbi8vIFlvdSBjYW4gdXNlIFJlZGlzIGlmIHlvdSBzZXQgdXAgc2Vzc2lvbiBzdG9yYWdlIHdpdGggUmVkaXNcclxuXHJcbmV4cG9ydCBhc3luYyBmdW5jdGlvbiBHRVQocmVxdWVzdDogTmV4dFJlcXVlc3QpIHtcclxuICAgIHRyeSB7XHJcbiAgICAgICAgLy8gQXNzdW1pbmcgeW91IGhhdmUgc3RvcmVkIHRoZSB1c2VybmFtZSBpbiBhIGNvb2tpZSBvciBzZXNzaW9uXHJcbiAgICAgICAgY29uc3QgY29va2llID0gcmVxdWVzdC5jb29raWVzLmdldCgndXNlcm5hbWUnKTsgLy8gRmV0Y2ggdXNlcm5hbWUgZnJvbSBjb29raWVcclxuXHJcbiAgICAgICAgaWYgKCFjb29raWUpIHtcclxuICAgICAgICAgICAgcmV0dXJuIE5leHRSZXNwb25zZS5qc29uKHsgZXJyb3I6ICdVc2VybmFtZSBub3QgZm91bmQnIH0sIHsgc3RhdHVzOiA0MDEgfSk7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBjb25zdCB1c2VybmFtZSA9IGNvb2tpZS52YWx1ZTsgLy8gRXh0cmFjdCB1c2VybmFtZSB2YWx1ZSBmcm9tIHRoZSBjb29raWVcclxuXHJcbiAgICAgICAgLy8gQ29ubmVjdCB0byBNb25nb0RCIGFuZCByZXRyaWV2ZSBkb2N1bWVudHNcclxuICAgICAgICBjb25zdCB7IGRiIH0gPSBhd2FpdCBjb25uZWN0VG9EYXRhYmFzZSgpO1xyXG4gICAgICAgIGNvbnN0IGNvbGxlY3Rpb25OYW1lID0gYCR7dXNlcm5hbWV9X3dhc3RlX3JlY29yZHNgO1xyXG4gICAgICAgIGNvbnNvbGUubG9nKFwiQ29sbGVjdGlvbiBuYW1lOlwiLCBjb2xsZWN0aW9uTmFtZSk7IC8vIERlYnVnZ2luZyBsaW5lXHJcblxyXG4gICAgICAgIGNvbnN0IGNvbGxlY3Rpb24gPSBkYi5jb2xsZWN0aW9uKGNvbGxlY3Rpb25OYW1lKTtcclxuICAgICAgICBjb25zdCBkb2N1bWVudHMgPSBhd2FpdCBjb2xsZWN0aW9uLmZpbmQoe30pLnRvQXJyYXkoKTtcclxuICAgICAgICBcclxuICAgICAgICByZXR1cm4gTmV4dFJlc3BvbnNlLmpzb24oZG9jdW1lbnRzKTtcclxuICAgIH0gY2F0Y2ggKGVycm9yKSB7XHJcbiAgICAgICAgY29uc29sZS5lcnJvcignRXJyb3IgZmV0Y2hpbmcgcmVjb3JkczonLCBlcnJvcik7XHJcbiAgICAgICAgcmV0dXJuIE5leHRSZXNwb25zZS5qc29uKHsgZXJyb3I6ICdGYWlsZWQgdG8gZmV0Y2ggcmVjb3JkcycgfSwgeyBzdGF0dXM6IDUwMCB9KTtcclxuICAgIH1cclxufVxyXG4iXSwibmFtZXMiOlsiTmV4dFJlc3BvbnNlIiwiY29ubmVjdFRvRGF0YWJhc2UiLCJHRVQiLCJyZXF1ZXN0IiwiY29va2llIiwiY29va2llcyIsImdldCIsImpzb24iLCJlcnJvciIsInN0YXR1cyIsInVzZXJuYW1lIiwidmFsdWUiLCJkYiIsImNvbGxlY3Rpb25OYW1lIiwiY29uc29sZSIsImxvZyIsImNvbGxlY3Rpb24iLCJkb2N1bWVudHMiLCJmaW5kIiwidG9BcnJheSJdLCJpZ25vcmVMaXN0IjpbXSwic291cmNlUm9vdCI6IiJ9\n//# sourceURL=webpack-internal:///(rsc)/./app/api/users/current/route.ts\n");

/***/ }),

/***/ "(rsc)/./utils/mongodb.ts":
/*!**************************!*\
  !*** ./utils/mongodb.ts ***!
  \**************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   connectToDatabase: () => (/* binding */ connectToDatabase)\n/* harmony export */ });\n/* harmony import */ var mongodb__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! mongodb */ \"mongodb\");\n/* harmony import */ var mongodb__WEBPACK_IMPORTED_MODULE_0___default = /*#__PURE__*/__webpack_require__.n(mongodb__WEBPACK_IMPORTED_MODULE_0__);\n\nlet client = null;\nlet db = null;\nconst uri = process.env.MONGODB_URI; // MongoDB connection string from environment variables\nconst dbName = process.env.MONGODB_DB; // Database name from environment variables\nif (!uri || !dbName) {\n    throw new Error('Please define the MONGODB_URI and MONGODB_DB environment variables');\n}\nasync function connectToDatabase() {\n    if (client && db) {\n        // Return existing connection if available\n        return {\n            client,\n            db\n        };\n    }\n    // Otherwise, create a new client instance and connect\n    client = new mongodb__WEBPACK_IMPORTED_MODULE_0__.MongoClient(uri);\n    await client.connect();\n    db = client.db(dbName);\n    console.log('Connected to MongoDB');\n    return {\n        client,\n        db\n    };\n}\n//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHJzYykvLi91dGlscy9tb25nb2RiLnRzIiwibWFwcGluZ3MiOiI7Ozs7OztBQUEwQztBQUUxQyxJQUFJQyxTQUE2QjtBQUNqQyxJQUFJQyxLQUFnQjtBQUNwQixNQUFNQyxNQUFNQyxRQUFRQyxHQUFHLENBQUNDLFdBQVcsRUFBWSx1REFBdUQ7QUFDdEcsTUFBTUMsU0FBU0gsUUFBUUMsR0FBRyxDQUFDRyxVQUFVLEVBQVksMkNBQTJDO0FBRTVGLElBQUksQ0FBQ0wsT0FBTyxDQUFDSSxRQUFRO0lBQ25CLE1BQU0sSUFBSUUsTUFBTTtBQUNsQjtBQUVPLGVBQWVDO0lBQ3BCLElBQUlULFVBQVVDLElBQUk7UUFDaEIsMENBQTBDO1FBQzFDLE9BQU87WUFBRUQ7WUFBUUM7UUFBRztJQUN0QjtJQUVBLHNEQUFzRDtJQUN0REQsU0FBUyxJQUFJRCxnREFBV0EsQ0FBQ0c7SUFDekIsTUFBTUYsT0FBT1UsT0FBTztJQUNwQlQsS0FBS0QsT0FBT0MsRUFBRSxDQUFDSztJQUVmSyxRQUFRQyxHQUFHLENBQUM7SUFFWixPQUFPO1FBQUVaO1FBQVFDO0lBQUc7QUFDdEIiLCJzb3VyY2VzIjpbIkM6XFxVc2Vyc1xcQXJqdW5cXERlc2t0b3BcXEJhbmtFYXNlIENvcGllc1xcRWNvLURleFxcV2ViRGV2XFx1dGlsc1xcbW9uZ29kYi50cyJdLCJzb3VyY2VzQ29udGVudCI6WyJpbXBvcnQgeyBNb25nb0NsaWVudCwgRGIgfSBmcm9tICdtb25nb2RiJztcclxuXHJcbmxldCBjbGllbnQ6IE1vbmdvQ2xpZW50IHwgbnVsbCA9IG51bGw7XHJcbmxldCBkYjogRGIgfCBudWxsID0gbnVsbDtcclxuY29uc3QgdXJpID0gcHJvY2Vzcy5lbnYuTU9OR09EQl9VUkkgYXMgc3RyaW5nOyAvLyBNb25nb0RCIGNvbm5lY3Rpb24gc3RyaW5nIGZyb20gZW52aXJvbm1lbnQgdmFyaWFibGVzXHJcbmNvbnN0IGRiTmFtZSA9IHByb2Nlc3MuZW52Lk1PTkdPREJfREIgYXMgc3RyaW5nOyAvLyBEYXRhYmFzZSBuYW1lIGZyb20gZW52aXJvbm1lbnQgdmFyaWFibGVzXHJcblxyXG5pZiAoIXVyaSB8fCAhZGJOYW1lKSB7XHJcbiAgdGhyb3cgbmV3IEVycm9yKCdQbGVhc2UgZGVmaW5lIHRoZSBNT05HT0RCX1VSSSBhbmQgTU9OR09EQl9EQiBlbnZpcm9ubWVudCB2YXJpYWJsZXMnKTtcclxufVxyXG5cclxuZXhwb3J0IGFzeW5jIGZ1bmN0aW9uIGNvbm5lY3RUb0RhdGFiYXNlKCkge1xyXG4gIGlmIChjbGllbnQgJiYgZGIpIHtcclxuICAgIC8vIFJldHVybiBleGlzdGluZyBjb25uZWN0aW9uIGlmIGF2YWlsYWJsZVxyXG4gICAgcmV0dXJuIHsgY2xpZW50LCBkYiB9O1xyXG4gIH1cclxuXHJcbiAgLy8gT3RoZXJ3aXNlLCBjcmVhdGUgYSBuZXcgY2xpZW50IGluc3RhbmNlIGFuZCBjb25uZWN0XHJcbiAgY2xpZW50ID0gbmV3IE1vbmdvQ2xpZW50KHVyaSk7XHJcbiAgYXdhaXQgY2xpZW50LmNvbm5lY3QoKTtcclxuICBkYiA9IGNsaWVudC5kYihkYk5hbWUpO1xyXG5cclxuICBjb25zb2xlLmxvZygnQ29ubmVjdGVkIHRvIE1vbmdvREInKTtcclxuXHJcbiAgcmV0dXJuIHsgY2xpZW50LCBkYiB9O1xyXG59XHJcbiJdLCJuYW1lcyI6WyJNb25nb0NsaWVudCIsImNsaWVudCIsImRiIiwidXJpIiwicHJvY2VzcyIsImVudiIsIk1PTkdPREJfVVJJIiwiZGJOYW1lIiwiTU9OR09EQl9EQiIsIkVycm9yIiwiY29ubmVjdFRvRGF0YWJhc2UiLCJjb25uZWN0IiwiY29uc29sZSIsImxvZyJdLCJpZ25vcmVMaXN0IjpbXSwic291cmNlUm9vdCI6IiJ9\n//# sourceURL=webpack-internal:///(rsc)/./utils/mongodb.ts\n");

/***/ })

};
;

// load runtime
var __webpack_require__ = require("../../../../webpack-runtime.js");
__webpack_require__.C(exports);
var __webpack_exec__ = (moduleId) => (__webpack_require__(__webpack_require__.s = moduleId))
var __webpack_exports__ = __webpack_require__.X(0, ["vendor-chunks/next"], () => (__webpack_exec__("(rsc)/./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fusers%2Fcurrent%2Froute&page=%2Fapi%2Fusers%2Fcurrent%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fusers%2Fcurrent%2Froute.ts&appDir=C%3A%5CUsers%5CArjun%5CDesktop%5CBankEase%20Copies%5CEco-Dex%5CWebDev%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=C%3A%5CUsers%5CArjun%5CDesktop%5CBankEase%20Copies%5CEco-Dex%5CWebDev&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D!")));
module.exports = __webpack_exports__;

})();