#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>

#include "libroxml/inc/roxml-internal.h"
#include "xml_utils.h"
#include "common.h"

#include "os_utils.h"
#include "string_utils.h"

#define XML_RUN_PATH "temp"

pthread_mutex_t xml_mutex = PTHREAD_MUTEX_INITIALIZER;

xml_node* xml_add_node(xml_node* n, int position, int type, char* name, char* value) {
	xml_node* new_node = NULL;

	if (name != NULL) {
		pthread_mutex_lock(&xml_mutex);
		new_node = roxml_add_node(n, position, type, name, value);
		pthread_mutex_unlock(&xml_mutex);
	}

	return new_node;
}

void xml_del_node(xml_node* n) {
	if (n) {
		pthread_mutex_lock(&xml_mutex);
		roxml_del_node(n);
		pthread_mutex_unlock(&xml_mutex);
	}
}

int xml_set_attr(xml_node* n, char* name, char* value) {
	if (n == NULL || name == NULL || value == NULL)
		return -1;
	pthread_mutex_lock(&xml_mutex);
	if (roxml_get_type(n) != ROXML_ELM_NODE) {
		pthread_mutex_unlock(&xml_mutex);
		return -1;
	}

	xml_node* attrNode = roxml_get_attr(n, name, 0);
	if (attrNode) {
		xml_node* txtNode = roxml_get_txt(attrNode, 0);

		if (txtNode)
			roxml_del_node(txtNode);
		roxml_add_node(attrNode, 0, ROXML_TXT_NODE, NULL, value);

	}
	pthread_mutex_unlock(&xml_mutex);
	return 0;
}

xml_node* xml_new_attr(xml_node* n, char* name, char* value) {
	if (n == NULL || name == NULL)
		return NULL;

	pthread_mutex_lock(&xml_mutex);

	if (roxml_get_type(n) != ROXML_ELM_NODE) {

		pthread_mutex_unlock(&xml_mutex);
		return NULL;
	}

	xml_node* node = roxml_add_node(n, 0, ROXML_ATTR_NODE, name, value);
	pthread_mutex_unlock(&xml_mutex);
	return node;
}

xml_node* xml_has_attr(xml_node* n, char* name) {
	if (n == NULL || name == NULL)
		return NULL;
	pthread_mutex_lock(&xml_mutex);
	xml_node* attrNode = roxml_get_attr(n, name, 0);
	pthread_mutex_unlock(&xml_mutex);
	return attrNode;
}

int xml_set_content(xml_node* n, char* value) {
	int ret = 0;
	pthread_mutex_lock(&xml_mutex);
	if (n == NULL || value == NULL) {
		ret = -1;
	} else if (roxml_get_type(n) != ROXML_ELM_NODE) {
		ret = -1;
	} else {
		xml_node* txtNode = roxml_get_txt(n, 0);
		if (txtNode)
			roxml_del_node(txtNode);
		roxml_add_node(n, 0, ROXML_TXT_NODE, NULL, value);
	}
	pthread_mutex_unlock(&xml_mutex);

	return ret;
}

int xml_save_doc(xml_doc* root, char* file) {

	if (root == NULL) {
		fprintf(stderr, "null root\n");
		return -1;
	}

	if (file == NULL) {
		return -2;
	}

	pthread_mutex_lock(&xml_mutex);

	int commitRet = roxml_commit_changes(root, file, NULL, 1);

	pthread_mutex_unlock(&xml_mutex);

	if (commitRet <= 0) {
		return -3;
	}

	return 0;
}

xml_node* _xml_get_nodes(xml_node* n, int type, char* name, int nth) {
	pthread_mutex_lock(&xml_mutex);
	xml_node* node = roxml_get_nodes(n, type, name, nth);
	pthread_mutex_unlock(&xml_mutex);
	return node;
}

char* xml_get_attr(xml_node* n, const char* name, char* buf, int len, int* size) {
	if (n == NULL || name == NULL)
		return NULL;

	int atype = xml_get_type(n);
	if (atype != ROXML_ELM_NODE)
		return NULL;

    xml_node* attrNode = _xml_get_nodes(n, ROXML_ATTR_NODE, (char*)name, 0);

	if (NULL != attrNode) {
		if (buf) {
			xml_get_content(attrNode, buf, len, size);
		} else {
			return xml_get_content(attrNode, NULL, 0, NULL);
		}
	}
	return NULL;
}

int xml_get_path(xml_node* n, char* node_path) {
	if (n == NULL)
		return -1;

	memset((void*) node_path, 0, sizeof(node_path));

	pthread_mutex_lock(&xml_mutex);
	xml_node* cur = n;
	char* name = roxml_get_name(cur, NULL, 0);

	if (name == NULL) {
		pthread_mutex_unlock(&xml_mutex);
		return -1;
	}
	char tmp[STRING_MID_LENGTH] = { 0 };

	sprintf(node_path, "%s", name);

	//printf("-------> node_path: %s\n", node_path);
	roxml_release(name);

	xml_node* parent = roxml_get_parent(cur);
	while (parent) {
		name = roxml_get_name(parent, NULL, 0);
		//printf("-------> name1: %s\n", name);
		if (name) {
			strcpy(tmp, node_path);

			if (!strcmp("documentRoot", name)) {
				roxml_release(name);
				break;
			}

			sprintf(node_path, "%s/%s", name, tmp);
			//printf("-------> name2: %s\n", node_path);
			roxml_release(name);
		}
		cur = parent;
		parent = roxml_get_parent(cur);
	}
	sprintf(node_path, "/%s", tmp);
	//printf("-------> name3: %s\n", node_path);
	pthread_mutex_unlock(&xml_mutex);
	return 0;
}

int xml_get_chld_nb(xml_node* n) {
	pthread_mutex_lock(&xml_mutex);
	int nb = roxml_get_chld_nb(n);
	pthread_mutex_unlock(&xml_mutex);
	return nb;
}

int xml_get_attr_nb(xml_node* n) {
	pthread_mutex_lock(&xml_mutex);
	int nb = roxml_get_attr_nb(n);
	pthread_mutex_unlock(&xml_mutex);
	return nb;
}

void _xml_get_child_nb_recursive(xml_node* n, int *count) {

	int nb = xml_get_chld_nb(n);
	int i = 0;

	(*count) += nb;
	for (i = 0; i < nb; i++) {
		xml_node* cur = xml_get_chld(n, NULL, i);
		_xml_get_child_nb_recursive(cur, count);
	}
}

int xml_get_child_nb(xml_node* n, int extended) {
	int count = 0;

	switch (extended) {
	case 0: {
		_xml_get_child_nb_recursive(n, &count);
		break;
	}
	case 1:
		count = xml_get_chld_nb(n);
		break;
	}

	return count;
}

void _xml_get_child_recursive(xml_node* n, int *id, xml_node** node_set) {

	int nb = xml_get_chld_nb(n);
	int i = 0;

	for (i = 0; i < nb; i++) {
		xml_node* cur = xml_get_chld(n, NULL, i);
		node_set[*id] = cur;
		(*id)++;
		_xml_get_child_recursive(cur, id, node_set);
	}
}
/*
 * parameters:
 * 		n: node pointer
 * 		size: return number
 * 		extended: 0: all children, 1:one level(default)
 *
 * return:
 * 		children array pointer
 *
 * The caller must free the return pointer. free()
 */
xml_node** xml_get_chlds(xml_node* n, int* nb, int extended) {
	if (n == NULL || (xml_get_type(n) != ROXML_ELM_NODE)) {
		return NULL;
	}

	int i = 0;
	int id = 0;

	*nb = xml_get_child_nb(n, extended);

	if (*nb == 0)
		return NULL;

	xml_node** node_set = malloc(sizeof(xml_node*) * (*nb));

	switch (extended) {
	case 0: {
		_xml_get_child_recursive(n, &id, node_set);
		break;
	}
	case 1:
	default:
		for (i = 0; i < *nb; i++) {
			xml_node* cur = xml_get_chld(n, NULL, i);
			node_set[i] = cur;
		}
		break;
	}

	return node_set;
}

/*
 * parameters:
 * 		n: node pointer
 * 		extended: 0: include children (namespace, attributes & children), 1:self (namespace, attributes)
 * return:
 * 		new node pointer
 */

void _xml_copy_child_recurive(xml_node* n, xml_node* nnode) {
	int child_nb = xml_get_chld_nb(n);
	int i, j = 0;
	char name[STRING_MAX2_LENGTH] = { 0 };
	char content[STRING_MAX_LENGTH] = { 0 };

	for (i = 0; i < child_nb; i++) {
		xml_node* cur = xml_get_chld(n, NULL, i);
		xml_get_name(cur, name, sizeof(name));
		xml_get_content(cur, content, sizeof(content), NULL);

		xml_node* cnode = xml_add_node(nnode, 0, ROXML_ELM_NODE, name, content);
		int attr_nb = xml_get_attr_nb(cur);
		for (j = 0; j < attr_nb; j++) {
			xml_node* attr = xml_get_attrNode(cur, NULL, j);
			xml_get_name(attr, name, sizeof(name));
			xml_get_content(attr, content, sizeof(content), NULL);

			xml_add_node(cnode, 0, ROXML_ATTR_NODE, name, content);
			xml_release(attr);
		}
		_xml_copy_child_recurive(cur, cnode);
	}
}

xml_node* xml_copy_node(xml_node* n, int extended) {

	int i = 0;
	int nb = 0;

	xml_node* new_node = 0;
	char name[STRING_MAX2_LENGTH] = { 0 };
	char content[STRING_MAX_LENGTH] = { 0 };

	xml_get_name(n, name, sizeof(name));
	xml_get_content(n, content, sizeof(content), NULL);

	switch (extended) {
	case 0:
		new_node = xml_add_node(NULL, 0, ROXML_ELM_NODE, name, content);
		nb = xml_get_attr_nb(n);
		for (i = 0; i < nb; i++) {
			xml_node* attr = xml_get_attrNode(n, NULL, i);
			xml_get_name(attr, name, sizeof(name));
			xml_get_content(attr, content, sizeof(content), NULL);
			xml_add_node(new_node, 0, ROXML_ATTR_NODE, name, content);
			xml_release(attr);
		}
		_xml_copy_child_recurive(n, new_node);

		break;
	case 1:
		new_node = xml_add_node(NULL, 0, ROXML_ELM_NODE, name, content);
		nb = xml_get_attr_nb(n);
		for (i = 0; i < nb; i++) {
			xml_node* attr = xml_get_attrNode(n, NULL, i);
			xml_get_name(attr, name, sizeof(name));
			xml_get_content(attr, content, sizeof(content), NULL);
			xml_add_node(new_node, 0, ROXML_ATTR_NODE, name, content);
			xml_release(attr);
		}

		break;
	}

	return new_node;
}

void xml_add_chld(xml_node* parent, xml_node* child, int extended) {

	char name[STRING_MAX2_LENGTH] = { 0 };
	char content[STRING_MAX_LENGTH] = { 0 };

	xml_get_name(child, name, sizeof(name));
	xml_get_content(child, content, sizeof(content), NULL);

	xml_node* nnode = xml_add_node(parent, 0, ROXML_ELM_NODE, name, content);
	int nb = xml_get_attr_nb(child);

	int i;
	for (i = 0; i < nb; i++) {
		xml_node* attr = xml_get_attrNode(child, NULL, i);

		xml_get_name(attr, name, sizeof(name));
		xml_get_content(attr, content, sizeof(content), NULL);

		xml_add_node(nnode, 0, ROXML_ATTR_NODE, name, content);

		xml_release(attr);
	}

	if (extended == 0) {
		_xml_copy_child_recurive(child, nnode);
	}
}

int xml_set_name(xml_node* n, char* value) {
	if (n == NULL || value == NULL)
		return -1;

	if (xml_get_type(n) != ROXML_ELM_NODE)
		return -1;

	int i = 0;
	xml_node* new_node;

	char name[STRING_MAX2_LENGTH] = { 0 };
	char content[STRING_MAX_LENGTH] = { 0 };
	xml_node* parent = xml_get_parent(n);
	xml_get_content(n, content, sizeof(content), NULL);

	new_node = xml_add_node(NULL, 0, ROXML_ELM_NODE, value, content);
	int nb = xml_get_attr_nb(n);
	for (i = 0; i < nb; i++) {
		xml_node* attr = xml_get_attrNode(n, NULL, i);
		xml_get_name(attr, name, sizeof(name));
		xml_get_content(attr, content, sizeof(content), NULL);
		xml_add_node(new_node, 0, ROXML_ATTR_NODE, name, content);
		//roxml_release(attr);
	}
	_xml_copy_child_recurive(n, new_node);

	xml_add_chld(parent, new_node, 0);

	xml_del_node(n);

	return 0;
}

// modified from roxml_set_ns
xml_node* _roxml_set_ns_no_recursif(xml_node *n, xml_node * ns) {
	xml_node * attr = NULL;

	if (!n || !ns) {
		return NULL;
	}

	pthread_mutex_lock(&xml_mutex);
	n->ns = ns;
	attr = n->attr;
	while (attr) {
		if ((attr->type & ROXML_NS_NODE) == 0) {
			attr->ns = ns;
		}
		attr = attr->sibl;
	}
	pthread_mutex_unlock(&xml_mutex);
	return n;
}

xml_node* _xml_set_ns(xml_node* n, char* value) {

	xml_node* new_node;

	char name[STRING_MIN_LENGTH] = { 0 };
	char content[STRING_MID_LENGTH] = { 0 };

	xml_node* ns_n = roxml_get_ns(n);
	xml_get_name(ns_n, name, sizeof(name));
	xml_get_content(ns_n, content, sizeof(content), NULL);

	//printf("name, content = %s, %s\n", name, content);

	new_node = xml_add_node(n, 0, ROXML_NSDEF_NODE, value, content);

	_roxml_set_ns_no_recursif(n, new_node);

	return n;
}
/*
 * <SOAP-ENV:Envelope
 * xmlns:SOAP-ENV=\"http://schemas.xmlsoap.org/soap/envelope/\"
 * xmlns:SOAP-ENC=\"http://schemas.xmlsoap.org/soap/encoding/\"
 * xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\"
 * xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"
 * xmlns:cwmp=\"urn:dslforum-org:cwmp-1-1\">
 * 	<SOAP-ENV:Header>
 * 		<cwmp:ID SOAP-ENV:mustUnderstand=\"1\">1000015432_7B2A32F34D77E523CF1358486382197150361</cwmp:ID>
 * 	</SOAP-ENV:Header>
 * 	<SOAP-ENV:Body>
 * 		<cwmp:GetParameterNames>
 * 			<ParameterPath>InternetGatewayDevice.DeviceInfo.HardwareVersion</ParameterPath>
 * 			<NextLevel>1</NextLevel>
 * 		</cwmp:GetParameterNames>
 * 	</SOAP-ENV:Body>
 * </SOAP-ENV:Envelope>"
 *
 */
/*
 * xmlXPathRegisterNs(xpathCtx, (xmlChar*)"soap",		(xmlChar*)"http://schemas.xmlsoap.org/soap/envelope/");
 * xmlXPathRegisterNs(xpathCtx, (xmlChar*)"encoding",	(xmlChar*)"http://schemas.xmlsoap.org/soap/encoding/");
 * xmlXPathRegisterNs(xpathCtx, (xmlChar*)"cwmp",		(xmlChar*)"urn:dslforum-org:cwmp-1-1");
 * xpathObj = xmlXPathEval((xmlChar*)"/soap:Envelope/soap:Header/cwmp:ID", xpathCtx);
 * ==>
 * ans = roxml_xpath(doc, "/SOAP-ENV:Envelope/SOAP-ENV:Header/cwmp:ID", &nb);
 */
void xml_register_ns(xml_node* ctx, char* name, char* ns) {
	char buf[STRING_MID_LENGTH];

	int nb;
	xml_node** ans = NULL;
	int i = 0;

	memset(buf, 0, sizeof(buf));
	sprintf(buf, ".//*[namespace-uri()='%s'", ns);
	//printf("buf = %s\n", buf);
	ans = xml_xpath(ctx, buf, &nb);

	//printf("nb = %d\n", nb);

	for (i = 0; i < nb; i++) {
		_xml_set_ns(ans[i], name);
	}

	xml_release(ans);
}

void strip(char *s) {
	char *p2 = s;
	while (*s != '\0') {
		if (*s == '\n' || *s == '\t' || *s == '\r') {
			++s;
			while (*s == ' ') {
				++s;
			}
		} else {
			*p2++ = *s++;
		}
	}
	*p2 = '\0';
}

char* xml_get_content(xml_node* n, char* buffer, int size, int* cnt) {
	char* content = NULL;
	pthread_mutex_lock(&xml_mutex);
	content = roxml_get_content(n, buffer, size, cnt);

	if (buffer != NULL) {
		strip(buffer);
		if (cnt != NULL)
			*cnt = strlen(buffer);
	}

	if (content != NULL) {
		strip(content);
		if (*content == '\0') {
			roxml_release(content);
			pthread_mutex_unlock(&xml_mutex);
			return NULL;
		}
	}
	pthread_mutex_unlock(&xml_mutex);
	return content;
}

xml_node** xml_xpath(xml_node* n, char* path, int* count) {
	pthread_mutex_lock(&xml_mutex);
	xml_node** ans = roxml_xpath(n, path, count);
	pthread_mutex_unlock(&xml_mutex);
	return ans;
}

xml_node* xml_load_doc(char* dirpath, char* filename) {

	if (filename == NULL )
		return NULL;

    //  mkdir "Profile/run/"
	if (OSUtils_isFileExisted(XML_RUN_PATH) == false) {
		bool mdRet = OSUtils_mkdir(XML_RUN_PATH);
		if (mdRet == false) {
			printf("mkdir xml run dir [%s] error!\n", XML_RUN_PATH);
			return NULL;
		}
	}

    //  copy "*.xml" to "temp/"
	char xmlSrcFilePath[257] = { 0 };
	char xmlRunFilePath[257] = { 0 };

	if(dirpath == NULL || strlen(dirpath) == 0 )
		sprintf(xmlSrcFilePath, "%s", filename);
	else{
#ifdef WIN32
		if(strstr(dirpath, "/") != NULL  ){
			char* path = String_replace(dirpath, "/", "\\");
			sprintf(xmlSrcFilePath, "%s\\%s", path, filename);
			SAFE_FREE(path);
		}else
			sprintf(xmlSrcFilePath, "%s\\%s", dirpath, filename);
#else
		sprintf(xmlSrcFilePath, "%s/%s", dirpath, filename);
#endif
	}

	OSUtils_copy(FOT_SINGLE, xmlSrcFilePath, XML_RUN_PATH);

	sprintf(xmlRunFilePath, "%s/%s", XML_RUN_PATH, filename);

	pthread_mutex_lock(&xml_mutex);
	xml_node* doc = roxml_load_doc(xmlRunFilePath);
	pthread_mutex_unlock(&xml_mutex);
	return doc;
}

xml_node* xml_load_buf(char* buffer) {
	pthread_mutex_lock(&xml_mutex);
	xml_node* doc = roxml_load_buf(buffer);
	pthread_mutex_unlock(&xml_mutex);
	return doc;
}

char* xml_get_name(xml_node* n, char* buffer, int size) {
	pthread_mutex_lock(&xml_mutex);
	char* name = roxml_get_name(n, buffer, size);
	pthread_mutex_unlock(&xml_mutex);
	return name;
}

xml_node* xml_get_chld(xml_node* n, char* name, int nth) {
	pthread_mutex_lock(&xml_mutex);
	xml_node* child = roxml_get_chld(n, name, nth);
	pthread_mutex_unlock(&xml_mutex);
	return child;
}

int xml_get_type(xml_node* n) {
	pthread_mutex_lock(&xml_mutex);
	int type = roxml_get_type(n);
	pthread_mutex_unlock(&xml_mutex);
	return type;
}

xml_node* xml_get_parent(xml_node* n) {
	pthread_mutex_lock(&xml_mutex);
	xml_node* parent = roxml_get_parent(n);
	pthread_mutex_unlock(&xml_mutex);
	return parent;
}

xml_node* xml_get_next_sibling(xml_node* n) {
	pthread_mutex_lock(&xml_mutex);
	xml_node* node = roxml_get_next_sibling(n);
	pthread_mutex_unlock(&xml_mutex);
	return node;
}

void xml_close(xml_node* n) {
	if (n == NULL)
		return;

    pthread_mutex_lock(&xml_mutex);
	roxml_close(n);
    pthread_mutex_unlock(&xml_mutex);
}

void xml_release(void* data) {
	if (data == NULL)
		return;

	pthread_mutex_lock(&xml_mutex);
	roxml_release(data);
	pthread_mutex_unlock(&xml_mutex);
}

xml_node* xml_get_attrNode(xml_node* n, char* name, int nth) {
	pthread_mutex_lock(&xml_mutex);
	xml_node* node = roxml_get_attr(n, name, nth);
	pthread_mutex_unlock(&xml_mutex);
	return node;
}

int xml_getAttributeFromRoot(xml_node* root, char* nodeXPath, char* attributeName, char* result, int resultSize) {
    xml_node*  node;
    xml_node** nodeSearchResults = NULL;
    int        intCount = 0;

    nodeSearchResults = xml_xpath(root, nodeXPath, &intCount);

    if (intCount > 0) {
        node   = nodeSearchResults[0];
        xml_get_attr(node, attributeName, result, resultSize, &intCount);
    } else
        intCount = -1;

    return intCount;
}

xml_node* xml_getSibling(xml_node* currentNode, const char* siblingName) {
    xml_node* nodeParent = xml_get_parent(currentNode);

    if (nodeParent == NULL)
        return NULL;

    xml_node* targetNode = xml_get_chld(nodeParent, (char*)siblingName, 0);
    return targetNode;
}
